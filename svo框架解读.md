# 1.函数如何进入

## 	1.1 进入 svo::BenchmarkNode benchmark,使用构造函数来初始化系统。

bunchmarknode的初始化函数	

```c++
BenchmarkNode::BenchmarkNode()
{
  cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}
```

初始化阶段分别初始化：

### 1.1.1相机模型 

初始化相机的相关功能，例如给每个参数赋值，以及相机-世界的坐标变换关系，相机的误差模型等等

### 1.1.2 frame handler，每一帧图像的处理句柄

==转入frame_handler_mono.cpp==的frame handler构造函数

```
// 构造函数
FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam) :
  FrameHandlerBase(),  //基类
  cam_(cam), //相机模型
  reprojector_(cam_, map_), //重投影类, map_来自基类
  depth_filter_(NULL)  //深度滤波器
{
  initialize();
}
```

接下来是进入void FrameHandlerMono::initialize()函数

函数干了三件事情：1.初始化fast角点

```c++
feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
```

2.声明depth filter的回调函数，用来构建地图点

\* depth_filter_cb（point*, sigma）参数传入map_.point_candidates_->newCandidatePoint(_1, _2 )

```c++
 DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
```

3.创建深度滤波类，并启动深度滤波

```c++
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
  depth_filter_->startThread();
```

## 1.2 runFromFolder

### 1.2.1 read image

separate image file name,and timestamps

```c++
 std::vector<std::string> vstrImageFilenames;
  std::vector<double> vTimestamps;
  std::string filepath = std::string("/media/hyj/dataset/datasets/freiburg2_desk");
  std::string strFile = filepath + "/rgb.txt";
  LoadImages(strFile, vstrImageFilenames, vTimestamps);
```

1.2.2

start for loop,每一张图片都要经历这个过程，在被读取之后，由VO的addimage去处理每一张图片

```
for(int ni=0; ni<nImages; ni++)
  {
          std::string img_path = filepath+"/"+vstrImageFilenames[ni];
          img = cv::imread(img_path.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
          assert(!img.empty());

          //开始处理程序
          vo_->addImage(img, vTimestamps[ni]);

          if(vo_->lastFrame() != NULL)
          {

            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                        << "#Features: " << vo_->lastNumObservations() << " \n";

          }
  }
```

# 2.每一帧如何处理

在==frame_handler_mono.cpp==的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)中，需要两个参数，一个是图片，一个是时间戳

## 2.1addimage

2.1.1 清空状态，开始计时

```c++
/[ ***step 1*** ] 判断 stage_, 开始计时, 并且清空trash
  if(!startFrameProcessingCommon(timestamp))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  core_kfs_.clear();      //!< 相近的关键帧
  overlap_kfs_.clear();   //!< 第一个参数具有一定共视关系的关键帧, 第二个参数是该帧观察到多少个地图点
```

2.1.2 创建两个帧，一个是正常图片，一个是正常图片的克隆，用于指针共享==？？？？==；都要建立相对应的金字塔

```c++
//[ ***step 2*** ] 创建一个新的帧, 会构建图像金字塔
  SVO_START_TIMER("pyramid_creation");
  //* 这个 reset 是共享指针的
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp)); // 共享指针的 reset
  SVO_STOP_TIMER("pyramid_creation");
```

2.1.3 为每一帧选择处理方式

前两帧因为涉及到初始化的内容，所以要分别处理;剩下的统一处理，然后重定位单独处理

将结果传递给==res==，用于更新结果，结果有三种类型:关键帧/非关键帧/失败

```c++
enum UpdateResult {
    RESULT_NO_KEYFRAME,
    RESULT_IS_KEYFRAME,
    RESULT_FAILURE
  };
```



```c++
//[ ***step 3*** ] 根据stage_确定如何处理
  // process frame
  UpdateResult res = RESULT_FAILURE;
  if(stage_ == STAGE_DEFAULT_FRAME)
    res = processFrame();
  else if(stage_ == STAGE_SECOND_FRAME)
    res = processSecondFrame();
  else if(stage_ == STAGE_FIRST_FRAME)
    res = processFirstFrame();
  else if(stage_ == STAGE_RELOCALIZING)
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));
```

每更新完一次之后：新变旧；新重置

对每一帧的结果进行记录，输入：id,该帧状态，上一帧观察到的特征数量

```c++
 last_frame_ = new_frame_; 
 new_frame_.reset();

finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
```

finishFrameProcessingCommon

```c++
//控制台两个输出
SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
  SVO_LOG(dropout);
// 读取时间
  acc_frame_timings_.push_back(timer_.stop());
 // 观察到的特征计数
  if(stage_ == STAGE_DEFAULT_FRAME)
    acc_num_obs_.push_back(num_observations);
  num_obs_last_ = num_observations; //上一次的观测数
  SVO_STOP_TIMER("tot_time");
```



### A). processFirstFrame

处理第一帧图片

#### 1.先初始化坐标变换new_frame_->T_f_w_

此处的new_frame是上面对当前图片的克隆；因为把第一帧作为世界坐标基准所以被定义为[I,0]

#### 2.再初始化（光流法的第一帧处理）

```
//如果光流初始化第一帧失败，那么就返回该函数不存在关键帧
if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
  return RESULT_NO_KEYFRAME;
```

klt_homography_init_.addFirstFrame(new_frame_)转入==initialization.cpp==

==光流法的第一帧处理==

输入参数：该图片

##### **过程1：特征检测**

```c++
detectFeatures(frame_ref, px_ref_, f_ref_);
if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }
```

frame_ref：参考帧；

px_ref_：检测得到的特征点；

f_ref_：在归一化平面上的向量；

如果检测到的数量小于100个那么就判定失败

> ```c++
> void detectFeatures(
>     FramePtr frame,
>     vector<cv::Point2f>& px_vec,
>     vector<Vector3d>& f_vec)
> {
>   Features new_features;
>   【1】首先初始化一个对象，进入他的构造函数
>    feature_detection::FastDetector detector(frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
>   【2】开始进行角点的检测，==网格划分也是在此处进行==
>   detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);
>   // - f_vec是特征点经过相机光心反投影cam2world()
>   // - (X,Y,Z) = ((u - cx)/fx, (v - cy)/fy, 1.0)
>   // now for all maximum corners, initialize a new seed
>   【给2D、3D点赋值】得到像素平面上的点，以及归一化平面上的点
>   px_vec.clear(); px_vec.reserve(new_features.size());
>   f_vec.clear(); f_vec.reserve(new_features.size());
>   std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
>     px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
>     f_vec.push_back(ftr->f); 
>     delete ftr;
>   });
> }
> 
> ```
>
> feature_detection::FastDetector都在feature_detection.cpp有详细的定义，==网格划分也是在此处进行==
>
> 赋值阶段：
>
> px：是像素在图像平面上的点，金字塔层数为0，Vector2d
>
> f是一个vector3f的向量，代表的是图片上的像素在归一化坐标系上的点

##### 过程2：将新检测到的特征点加入跟踪序列中,返回成功标志

```c++
px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  return SUCCESS;

```

vector::insert

> | iterator insert(pos,first,last) | 在迭代器 pos 指定的位置之前，插入其他容器（不仅限于vector）中位于 [first,last) 区域的所有元素，并返回表示第一个新插入元素位置的迭代器。 |
> | ------------------------------- | ------------------------------------------------------------ |
> |                                 |                                                              |

#### 3.初始化成功

将当前帧设为关键帧，并且将该帧加入地图中去，==状态变化为第二帧STAGE_SECOND_FRAME==

```c++
new_frame_->setKeyframe();
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_SECOND_FRAME;
  SVO_INFO_STREAM("Init: Selected first frame.");
  return RESULT_IS_KEYFRAME;
```



### B).processSecondFrame

#### 1.光流法的第二帧处理

转入==initialization.cpp==的InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)函数

##### 过程1：trackklt

```
trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
```

> ```c++
> void trackKlt(
>     FramePtr frame_ref,
>     FramePtr frame_cur,
>     vector<cv::Point2f>& px_ref,
>     vector<cv::Point2f>& px_cur,
>     vector<Vector3d>& f_ref,
>     vector<Vector3d>& f_cur,
>     vector<double>& disparities)
> {
>   const double klt_win_size = 30.0; //搜索窗口的大小
>   const int klt_max_iter = 30;
>   const double klt_eps = 0.001;
>   vector<uchar> status;
>   vector<float> error;
>   vector<float> min_eig_vec;
>   // 迭代终止条件
>   cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
>   // 参数：status为特征点是否被找到，error为找到点的测量误差
>   cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
>                            px_ref, px_cur,
>                            status, error,
>                            cv::Size2i(klt_win_size, klt_win_size),
>                            4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);
> 
>   vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
>   vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
>   vector<Vector3d>::iterator f_ref_it = f_ref.begin();
>   f_cur.clear(); f_cur.reserve(px_cur.size());
>   disparities.clear(); disparities.reserve(px_cur.size());
> 
>   for(size_t i=0; px_ref_it != px_ref.end(); ++i)
>   {
>     // 没找到则删除
>     if(!status[i])
>     {
>       px_ref_it = px_ref.erase(px_ref_it);
>       px_cur_it = px_cur.erase(px_cur_it);
>       f_ref_it = f_ref.erase(f_ref_it);
>       continue;
>     }
>     // 转化为归一化平面上的点
>     f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
>   // 计算ref和cur之间移动的大小，光流向量模值大小
>     disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
>     ++px_ref_it;
>     ++px_cur_it;
>     ++f_ref_it;
>   }
> }
> ```
>
> 

如果跟踪的FAST角点太少，则失败！

```c++
if(disparities_.size() < Config::initMinTracked())
    return FAILURE;
```

如果视差运动太小，则不是关键帧

```c++
  //特征点（光流场）移动的距离的太小，则不是关键帧
  double disparity = vk::getMedian(disparities_); //运动方向中值，方向由当前帧特征点指向参考帧特征点
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;
```

##### 过程2：根据光流法两帧跟踪结果，来计算单应矩阵

```
  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_);
```

> ```c++
> void computeHomography(
>     const vector<Vector3d>& f_ref,
>     const vector<Vector3d>& f_cur,
>     double focal_length,
>     double reprojection_threshold,
>     vector<int>& inliers,
>     vector<Vector3d>& xyz_in_cur,
>     SE3& T_cur_from_ref)
> {
>   vector<Vector2d > uv_ref(f_ref.size());
>   vector<Vector2d > uv_cur(f_cur.size());
> //[***step 1***] 归一化平面坐标投影到像素坐标
>   for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
>   {
>     uv_ref[i] = vk::project2d(f_ref[i]);
>     uv_cur[i] = vk::project2d(f_cur[i]);
>   }
> 
> //[***step 2***] 构造单应类，从单应矩阵恢复R,t
>   //单应矩阵类构造
>   vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
>   // 通过分解H得到T
>   Homography.computeSE3fromMatches();
>   
> //[***step 3***] 把特征点三角化计算重投影误差，决定内点还是外点
>   // 注意这里的点的顺序变了，先cur后ref，深度是在f_cur中的
>   vector<int> outliers;
>   vk::computeInliers(f_cur, f_ref,
>                      Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
>                      reprojection_threshold, focal_length,
>                      xyz_in_cur, inliers, outliers);
>   T_cur_from_ref = Homography.T_c2_from_c1;
> }
> ```
>
> 

 // 内点数量少则初始化失败

```c++
 if(inliers_.size() < Config::initMinInliers())

 {

  SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");

  return FAILURE;

 }
```

##### 过程3：获得深度，计算尺度，运动

提取每一个点的深度

```c++
  vector<double> depth_vec;
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
```

计算平均深度与尺度，因为==Config::mapScale()=1，所以尺度为1/depth==

```c++
double scene_depth_median = vk::getMedian(depth_vec); //深度的中值（以计算H矩阵的特征点在归一化平面上为前提的深度）
double scale = Config::mapScale()/scene_depth_median; //将深度的中值进行归一化
```

计算运动

```c++
frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_; //求得当前帧的变换矩阵
  // 求得当前帧的平移变量
frame_cur->T_f_w_.translation() =
    -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

```



##### 过程4：建立3D地图点

先判断2D点是否在图像中间区域内，然后将3D点乘上对应尺度变换到世界坐标系下

建立该3D点的指针

创建出某帧图片的指针，内容分别是：图片指针，3D点的指针，特征点的像素，特征点的归一化坐标

创建出每一个3D点的被观测到的帧addFrameRef

new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0))



同样对参考帧做相同的动作

```c++
SE3 T_world_cur = frame_cur->T_f_w_.inverse(); //获得cur到world的变换
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
    // 提取出内点
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    // isInFrame判断是否在边缘10个像素之内，默认是0层金字塔, 深度为正
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale); //乘以尺度, 转到world下坐标
      Point* new_point = new Point(pos); 
      // frame_cur.get()得到指针
      // 创建特征点, 特征点加入到对应帧(cur & ref)
      Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
      frame_cur->addFeature(ftr_cur);
      // 每个点会被几个帧观测到, 因此给点增加参考帧, 点时世界坐标下的
      // 这里的特征点和点的意义是不同的
      new_point->addFrameRef(ftr_cur);

      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);
    }
  }
```

返回成功==return SUCCESS;==

#### 2.如果定义使用BA，则进行优化

```c++
ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);

```

BA的参数：当前关键帧，上一个关键帧，重投影下界（跳出条件），地图点

第一步：建立稀疏求解器SparseOptimizer

第二步：定义两个顶点：第一帧被看做固定的所以设为TURE。第二帧的SE3来调整

第三步：添加3D点作为顶点 v_pt；分别建立frame1，frame2的边

第四步：运行求解器

第五步：用优化后的R，t，3D point来覆盖旧的量

第六步：重投影误差太大的话就要予以删除

> ```c++
> void twoViewBA(
>     Frame* frame1,
>     Frame* frame2,
>     double reproj_thresh,
>     Map* map)
> {
>   // scale reprojection threshold in pixels to unit plane
>   reproj_thresh /= frame1->cam_->errorMultiplier2(); // 转化到单位平面
> 
>   // init g2o
>   //[ ***step 1*** ] 配置图优化的参数
>   g2o::SparseOptimizer optimizer;
>   setupG2o(&optimizer);
> 
>   list<EdgeContainerSE3> edges;
>   size_t v_id = 0;
> 
>   //[ ***step 2*** ] 增加两个相机位姿顶点
>   // New Keyframe Vertex 1: This Keyframe is set to fixed!
>   g2oFrameSE3* v_frame1 = createG2oFrameSE3(frame1, v_id++, true);
>   optimizer.addVertex(v_frame1);
> 
>   // New Keyframe Vertex 2
>   g2oFrameSE3* v_frame2 = createG2oFrameSE3(frame2, v_id++, false);
>   optimizer.addVertex(v_frame2);
> 
>   // Create Point Vertices
>   //[ ***step 3*** ] 增加地图点的顶点, 以及和相机所连成的边
>   for(Features::iterator it_ftr=frame1->fts_.begin(); it_ftr!=frame1->fts_.end(); ++it_ftr)
>   {
>     Point* pt = (*it_ftr)->point;
>     if(pt == NULL)
>       continue;
>     g2oPoint* v_pt = createG2oPoint(pt->pos_, v_id++, false);
>     optimizer.addVertex(v_pt);
>       
>     pt->v_pt_ = v_pt; //放入做临时的G2O定点
>     g2oEdgeSE3* e = createG2oEdgeSE3(v_frame1, v_pt, vk::project2d((*it_ftr)->f), true, reproj_thresh*Config::lobaRobustHuberWidth());
>     optimizer.addEdge(e);
>     edges.push_back(EdgeContainerSE3(e, frame1, *it_ftr)); // TODO feature now links to frame, so we can simplify edge container!
> 
>     // find at which index the second frame observes the point
>     //* 找到和该点所对应的在 frame2 上的特征点, 构成边
>     Feature* ftr_frame2 = pt->findFrameRef(frame2);
>     e = createG2oEdgeSE3(v_frame2, v_pt, vk::project2d(ftr_frame2->f), true, reproj_thresh*Config::lobaRobustHuberWidth());
>     optimizer.addEdge(e);
>     edges.push_back(EdgeContainerSE3(e, frame2, ftr_frame2));
>   }
> 
>   // Optimization
>   //[ ***step 4*** ] 运行求解器
>   double init_error, final_error;
>   runSparseBAOptimizer(&optimizer, Config::lobaNumIter(), init_error, final_error);
>   printf("2-View BA: Error before/after = %f / %f\n", init_error, final_error);
> 
>   //[ ***step 5*** ] 使用优化后的相机位姿和点位置进行更新
>   // Update Keyframe Positions
>   frame1->T_f_w_.rotation_matrix() = v_frame1->estimate().rotation().toRotationMatrix();
>   frame1->T_f_w_.translation() = v_frame1->estimate().translation();
>   frame2->T_f_w_.rotation_matrix() = v_frame2->estimate().rotation().toRotationMatrix();
>   frame2->T_f_w_.translation() = v_frame2->estimate().translation();
> 
>   // Update Mappoint Positions
>   for(Features::iterator it=frame1->fts_.begin(); it!=frame1->fts_.end(); ++it)
>   {
>     if((*it)->point == NULL)
>      continue;
>     (*it)->point->pos_ = (*it)->point->v_pt_->estimate();
>     (*it)->point->v_pt_ = NULL;
>   }
> 
>   //[ ***step 6*** ] 沿着重投影误差大的, 则从地图中删除, 从edge中删除
>   // Find Mappoints with too large reprojection error
>   const double reproj_thresh_squared = reproj_thresh*reproj_thresh;
>   size_t n_incorrect_edges = 0;
>   for(list<EdgeContainerSE3>::iterator it_e = edges.begin(); it_e != edges.end(); ++it_e)
>     if(it_e->edge->chi2() > reproj_thresh_squared)
>     {
>       if(it_e->feature->point != NULL)
>       {
>         map->safeDeletePoint(it_e->feature->point);
>         it_e->feature->point = NULL;
>       }
>       ++n_incorrect_edges;
>     }
> 
>   printf("2-View BA: Wrong edges =  %zu\n", n_incorrect_edges);
> }
> ```
>
> 

#### 3.把第二帧加入深度滤波

```c++
//[ ***step 3*** ] 设置第二帧为关键帧, 并计算中位深度, 最小深度, 加入到深度滤波中
  new_frame_->setKeyframe();
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);
```

#### 4.将第二帧加入地图，初始化光流跟踪

```c++
//[ ***step 4*** ] 将第二帧作为关键帧加入到地图中, 重置初始化的类, 
  // add frame to map
  map_.addKeyframe(new_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
  return RESULT_IS_KEYFRAME;
```



### C).processFrame

到这一步初始化完成，开始正式的进行每帧图片的处理

#### 1.更新初始位姿

```c++
 new_frame_->T_f_w_ = last_frame_->T_f_w_;
```

#### 2.开始运行直接法

##### 2.1 粗对准sparse image alignment

```c++
SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
```

构造函数

\* @ param:  int n_levels,  // = max_level = max_level_ 粗金字塔

​       int min_level,  // = min_level_ 细金字塔 

​       int n_iter,   // 迭代次数, 继承自vk::NLLSSolver

​       Method method,  // GN 和 LM

​       bool display,  // = display_

​       bool verbose);  // = verbose_ 继承来, 输出统计信息

> ```c++
> SparseImgAlign::SparseImgAlign(
>     int max_level, int min_level, int n_iter,
>     Method method, bool display, bool verbose) :
>         display_(display),
>         max_level_(max_level),
>         min_level_(min_level)
> {
>   //继承来的
>   n_iter_ = n_iter;
>   n_iter_init_ = n_iter_;
>   method_ = method;
>   verbose_ = verbose;
>   eps_ = 0.000001;  //精度
> }
> ```

运行

```
img_align.run(frame_ref_, frame_cur_);
```



> ```c++
> size_t SparseImgAlign::run(FramePtr ref_frame, FramePtr cur_frame)
> { 
> //[***step 1***] 复位NLLSSolver的变量值
>   reset();
> 
>   if(ref_frame->fts_.empty())
>   {
>     SVO_WARN_STREAM("SparseImgAlign: no features to track!");
>     return 0;
>   }
> 
>   ref_frame_ = ref_frame;
>   cur_frame_ = cur_frame;
> 
> //[***step 2***] 初始化cache
>   //存储每个特征点的patch, 大小 (features_size*patch_area(4*4))
>   ref_patch_cache_ = cv::Mat(ref_frame_->fts_.size(), patch_area_, CV_32F);
>   //存储每个所有patch的雅克比, 大小 (6*ref_patch_cache_.size)
>   jacobian_cache_.resize(Eigen::NoChange, ref_patch_cache_.rows*patch_area_);
>   //存储可见的特征点, 类型vector<bool> 大小 feature_size, 默认都为false
>   visible_fts_.resize(ref_patch_cache_.rows, false); // TODO: should it be reset at each level?
> 
> //[***step 3***] 获得从参考帧到当前帧之间的变换 
>   // T_cur_from_world = T_cur_from_ref * T_ref_from_world
>   // T_[to]_[from]  T_A_C = T_A_B * T_B_C
>   SE3 T_cur_from_ref(cur_frame_->T_f_w_ * ref_frame_->T_f_w_.inverse());
> 
> //[***step 4***] 在不同的金字塔层对T_c_r进行稀疏图像对齐优化, 由粗到精, 具有更好的初值
>   // 在4level到2level之间
>   for(level_=max_level_; level_>=min_level_; --level_)
>   {
>     mu_ = 0.1;
>     jacobian_cache_.setZero();
>     have_ref_patch_cache_ = false;
>     if(verbose_)
>       printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
>     optimize(T_cur_from_ref);
>   }
> 
> //[***step 5***] 利用求得的T_c_r 求得 T_c_w
>   cur_frame_->T_f_w_ = T_cur_from_ref * ref_frame_->T_f_w_;
> 
>   // n_meas_表示前一帧所有特征点块(feature patch)像素投影后在cur_frame中的像素个数。
>   // n_meas_/patch_area_表示特征点数
>   return n_meas_/patch_area_;
> }
> ```

步骤1：

初始化每个点的patch存储空间，即光度块的数据，大小为4$\times$4。

初始化每个点的雅可比的存储空间，大小为6*n

初始化每个可见特征点的位置

步骤2：计算参考帧到当前帧的坐标变换

步骤3：在不同的金字塔下，对图片进行对齐

步骤4：计算cur_frame_->T_f_w_

##### 2.2 feature alignment

将初始化得到的3D点投影到2D平面上，



##### 2.3对位姿和3D点进行二次优化



```c++
//【位姿的优化】
size_t sfba_n_edges_final; // 误差观测数目
  double sfba_thresh, sfba_error_init, sfba_error_final;
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
      new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
//* 如果观测的点过少也不可信
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;
//【3D的优化】
optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
```

#### 3.判断跟踪质量

```c++
//加入到 core_kfs 中用来 localBA, 根据 1.特征点数目 2.特征减少的数目, 判断跟踪质量, 筛选关键帧  
core_kfs_.insert(new_frame_);  //!!!
  setTrackingQuality(sfba_n_edges_final);
 if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    return RESULT_FAILURE;
  }
	//* 通过位姿变换到当前帧, 计算深度
  	double depth_mean, depth_min;
  	frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
//如果不要要关键帧，并且跟踪质量差，就把当前帧用于给深度滤波器更新用 
if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    // 加入到深度滤波线程
    depth_filter_->addFrame(new_frame_);
    return RESULT_NO_KEYFRAME;
  }
```

#### 4.在选择完关键帧之后把深度滤波后的点再次加入地图中

```c++
[ ***step 7*** ] 满足上面的条件就设置成关键帧(提取5个关键点), 把该帧加入到point的参考帧, 以及把地图点加入参考帧
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // new keyframe selected
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);
  //? 这个怎么又加入到 frame 里投影时候不是加过???
  //?答: 投影时候增加的是原来的地图点, 加入深度滤波之后又重新提取了特征点, 因此这是新的
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);
```

#### 5.关键帧加入到深度滤波线程, 会重新提取特征点, 进行计算深度直到收敛

```c++
depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);
//* 如果地图中关键帧过多, 则要删除一些, 清理深度滤波的种子点, 地图中的点
  // if limited number of keyframes, remove the one furthest apart
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }
```



#### 6.加入地图关键帧

```c++
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
```



### D).relocalizeFrame

# 3.深度滤波器如何运行

## 3.1 DepthFilter

seed构造函数,主要用于构造深度相关的参数

```c++
Seed::Seed(Feature* ftr, float depth_mean, float depth_min) :
    batch_id(batch_counter),
    id(seed_counter++),           // 每次新建一个种子就加一
    live_time(0),
    ftr(ftr),
    a(10),
    b(10),
    mu(1.0/depth_mean),           // 逆深度的均值
    z_range(1.0/depth_min),       // 逆深度的最大值
    sigma2(z_range*z_range/36)    // 99% 的概率在这个区间的协方差? 为啥这么求
{
}
```

DepthFilter构造函数

```c++
DepthFilter::DepthFilter(feature_detection::DetectorPtr feature_detector, callback_t seed_converged_cb) :
    feature_detector_(feature_detector),
    seed_converged_cb_(seed_converged_cb),      //? 收敛
    seeds_updating_halt_(false),
    thread_(NULL),
    new_keyframe_set_(false),
    new_keyframe_min_depth_(0.0),
    new_keyframe_mean_depth_(0.0)             //? 最大值? 均值?
{
  std::string file = "/home/gong/catkin_ws/src/rpg_svo/seeds.txt";
  f.open(file.c_str());
}
```



## 3.2 addFrame

## 3.3 addKeyframe

### 1.计算一像素带来的像素扰动

$$
\delta\beta=arctan\frac{1}{f}
$$

### 2.投影到当前帧判断是否位置正确

1/mu是深度z(因为用了逆深度来表示)，f是单位平面上的3D坐标；二者相乘就是完整的3D点

```c++
const Vector3d xyz_f(T_ref_cur.inverse()*(1.0/it->mu * it->ftr->f));

if(xyz_f.z() < 0.0)  {
      ++it; // behind the camera
      continue;
    }
    if(!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>())) {
      ++it; // point does not project in image
      continue;
    }
```

### 3.极线搜索

已知ref和cur的位姿, 进行块匹配, 最终计算ref上精确的深度；

具体策略为：

1.由估计深度确定极线的搜索范围

2.沿着极线进行块匹配

3.匹配到合适的进行三角化，再次确立真正的深度值

范围是$[\mu+\sigma,\mu-sigma]$,因为用了逆深度所以大小互换了

```c++
    float z_inv_min = it->mu + sqrt(it->sigma2);
    float z_inv_max = max(it->mu - sqrt(it->sigma2), 0.00000001f);
    double z;
    if(!matcher_.findEpipolarMatchDirect(
        *it->ftr->frame, *frame, *it->ftr, 1.0/it->mu, 1.0/z_inv_min, 1.0/z_inv_max, z))
    {
      it->b++; // increase outlier probability when no match was found
      ++it;
      ++n_failed_matches;
      continue;
    }
```

### 4.更新种子点

计算一个像素的扰动带来的深度误差

```c++
    // compute tau
    double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
    double tau_inverse = 0.5 * (1.0/max(0.0000001, z-tau) - 1.0/(z+tau));
```

对深度进行更新

```c++
updateSeed(1./z, tau_inverse*tau_inverse, &*it);
++n_updates;
```

在一个栅格范围内只初始化一个点

```c++
    //* 如果是关键帧, 把对齐得到的位置占据
    if(frame->isKeyframe())
    {
      // The feature detector should not initialize new seeds close to this location
      feature_detector_->setGridOccpuancy(matcher_.px_cur_);
    }
```



### 5.如果收敛，就添加新的地图点

```c++
  //[ ***step 7*** ] 判断收敛, 将新的深度转化为新的3D点, 给种子点, 传给回调函数, 并删除种子点
  //? 这个收敛条件有些不懂
    // if the seed has converged, we initialize a new candidate point and remove the seed
    if(sqrt(it->sigma2) < it->z_range/options_.seed_convergence_sigma2_thresh)
    {
      assert(it->ftr->point == NULL); // TODO this should not happen anymore
      Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0/it->mu)));
      Point* point = new Point(xyz_world, it->ftr);
      //! 在这之前 feature 都没有指向的point, 只有frame.
      it->ftr->point = point;
```



## 3.4 updateSeeds

```c++
void DepthFilter::updateSeed(const float x, const float tau2, Seed* seed)
{
  seed->live_time++;
  float norm_scale = sqrt(seed->sigma2 + tau2);
  if(std::isnan(norm_scale))
    return;
  //! N (mu, sigma^2 + tau^2)  
  boost::math::normal_distribution<float> nd(seed->mu, norm_scale);
  //! 1/s^2=1/sigma^2 + 1/tau^2
  float s2 = 1./(1./seed->sigma2 + 1./tau2);
  //! s2 * (mu/sigma^2 + x/tau^2)
  float m = s2*(seed->mu/seed->sigma2 + x/tau2);
  //! a/(a+b)*N ( x | mu, sigma^2 + tau^2)  
  float C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
  //! b/(a+b)*U(x)
  float C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
  //! C=C1+C2 , 归一化  
  float normalization_constant = C1 + C2;
  C1 /= normalization_constant;
  C2 /= normalization_constant;

  float f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
  float e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
          + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));
  //* 更新参数
  // update parameters
  float mu_new = C1*m+C2*seed->mu;
  seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
  seed->mu = mu_new;
  seed->a = (e-f)/(f-e/f);
  seed->b = seed->a*(1.0f-f)/f;
}
```



## 3.5 computeTau