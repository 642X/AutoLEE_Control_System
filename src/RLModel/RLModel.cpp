#include "RLModel.h"
#include "Logger.h"

namespace SIAT {

RLModel::RLModel() : loaded(false) {}

RLModel::~RLModel() {}

bool RLModel::loadModel(const std::string& modelPath) {
    try {
        module = torch::jit::load(modelPath);
        loaded = true;

        // warm-up
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(torch::rand({38}));
        for (int i = 0; i < 100; ++i) {
            module.forward(inputs);
        }

        LOGI("RLModel", "模型加载成功：" + modelPath);
    } catch (const c10::Error& e) {
        LOGE("RLModel", "模型加载失败：" + std::string(e.what()));
        loaded = false;
    }

    return loaded;
}

std::vector<double> RLModel::infer(const at::Tensor& input) {
    std::vector<double> output_vector;

    if (!loaded) {
        LOGE("RLModel", "模型未加载，无法推理");
        return output_vector;
    }

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input);

    try {
        at::Tensor output = module.forward(inputs).toTensor();

        if (!output.defined()) {
            LOGE("RLModel", "模型推理结果未定义");
            return output_vector;
        }

        auto output_contig = output.contiguous();  // 确保内存连续
        auto num_elements = output_contig.numel();
        output_vector.resize(num_elements);

        if (output_contig.scalar_type() == torch::kDouble) {
            std::memcpy(output_vector.data(), output_contig.data_ptr<double>(), num_elements * sizeof(double));
        } else if (output_contig.scalar_type() == torch::kFloat) {
            // 先拷贝 float，再转换成 double
            std::vector<float> temp(num_elements);
            std::memcpy(temp.data(), output_contig.data_ptr<float>(), num_elements * sizeof(float));
            std::transform(temp.begin(), temp.end(), output_vector.begin(),
                           [](float val) { return static_cast<double>(val); });
        } else {
            LOGE("RLModel", "不支持的输出类型：" + std::string(torch::toString(output_contig.scalar_type())));
            output_vector.clear();
        }

        LOGD("RLModel", "推理完成并转换为 std::vector<double>");
    } catch (const c10::Error& e) {
        LOGE("RLModel", "推理过程出错：" + std::string(e.what()));
    }

    return output_vector;
}


bool RLModel::isLoaded() const {
    return loaded;
}

} // namespace SIAT
