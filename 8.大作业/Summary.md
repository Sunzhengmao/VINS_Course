### 实现了DogLeg法
在**problem.cpp**中实现了DogLeg优化方法，该方法属于信赖域法
### 代码结构与逻辑
- **run_euroc.cpp**
  生成3个线程，分别为①imu、②img、③backend   
  imu与img为从euroc中读取数据的地址
- **system.cpp**
  对三个线程做初步处理
  ①把imu的acc、gyr等信息push到imu的queue中；
  ②对img提取特征点，并push到img的queue中；
  ③backend则按照**一帧+多个imu**的方式取出来，交给estimator.cpp
- **estimator.cpp**
  ①imu做预积分与积分
  ②img决定marg哪个帧，并做优化与滑窗
  &emsp;优化：对滑窗中的约束做优化，prior+imu+img；并更新prior；
  &emsp;滑窗：根据marg的策略对窗口信息进行滑动，对marg的帧、特征点进行处理