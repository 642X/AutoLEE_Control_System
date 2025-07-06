#ifndef RLMODEL_H
#define RLMODEL_H

#include <torch/script.h>
#include <string>
#include <vector>

namespace SIAT {

class RLModel {
public:
    RLModel();
    ~RLModel();

    // 加载模型
    bool loadModel(const std::string& modelPath);

    // 推理执行
    std::vector<double> infer(const at::Tensor& input);

    // 模型是否已加载
    bool isLoaded() const;

private:
    torch::jit::script::Module module;
    bool loaded;
};

} // namespace SIAT

#endif // RLMODEL_H
