## 信息矩阵不满秩的处理方法
1. LM算法直接给对角阵添加$\mu I$；
2. 给某个变量的信息矩阵添加单位阵$I$，我的理解是在原来约束的基础上，强行引入$Iδx = 0$，为了满足这个约束只能使$δx=0$；
3. orb、svo更直接，将信息矩阵设为无穷大或0，$0*Δx=0$或$∞Δx=0$；