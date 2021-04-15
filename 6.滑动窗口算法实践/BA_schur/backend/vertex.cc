#include "backend/vertex.h"
#include <iostream>

namespace myslam {
namespace backend {

unsigned long global_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension)
{
    /// 把顶点重新定义尺寸为其本身的维度，而不是自由度
    parameters_.resize(num_dimension, 1);

    /// 把输入的值付给local_dimension_, 否则就与向量维度(non_dof)保持一致
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;

    id_ = global_vertex_id++;

//    std::cout << "Vertex construct num_dimension: " << num_dimension
//              << " local_dimension: " << local_dimension << " id_: " << id_ << std::endl;
}

Vertex::~Vertex() {}

int Vertex::Dimension() const {
    return parameters_.rows();
}

int Vertex::LocalDimension() const {
    return local_dimension_;
}

void Vertex::Plus(const VecX &delta) {
    parameters_ += delta;
}

}
}