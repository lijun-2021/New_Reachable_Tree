#pragma once
#define COMPILER_MSVC
#define NOMINMAX

#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/framework/graph.pb.h"
#include <vector>
#include <string>
#include <memory>
  
class Tensor {
public:
	tensorflow::Status status;
	tensorflow::GraphDef graphdef;
	tensorflow::SessionOptions options;
	std::shared_ptr<tensorflow::Session> session;

public:
	Tensor(const char* modelPath = kNetWork) {
		//声明Session和GraphDef
		options.config.set_allow_soft_placement(true);
		session = std::shared_ptr<tensorflow::Session>(tensorflow::NewSession(options));
		status = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), modelPath, &graphdef);
		if (!status.ok()) {
			std::cout << status.ToString().c_str() << std::endl;
		}
		status = session->Create(graphdef);
		if (!status.ok()) {
			std::cout << status.ToString().c_str() << std::endl;
		}
	}

	float predict(std::vector<float>& inputdatas) {
		int dimension = inputdatas.size();
		tensorflow::Tensor input_tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({ 1, dimension })); //维数
		auto input_tensor_mapped = input_tensor.tensor<float, 2>();
		for (int i = 0; i < dimension; ++i) {
			input_tensor_mapped(0, i) = inputdatas[i];
		}
		//模型的输入输出layer
		std::string inputLayer = "inputs";
		std::string outputLayer = "pred";
		std::vector<tensorflow::Tensor> finalOutput;
		std::vector<std::pair<tensorflow::string, tensorflow::Tensor> > inputs;
		inputs.push_back(std::make_pair(inputLayer, input_tensor));
		status = session->Run(inputs, { outputLayer }, {}, &finalOutput);
		if (!status.ok()) {
			std::cout << status.ToString().c_str() << std::endl;
			return -1;
		}
		auto output_y = finalOutput[0].scalar<float>();
		float h;
		h = output_y();
		return h;
	}

	float scale(float num, float mean = 0.0, float std = 1.0)  //通过标准化处理，可以使得不同的特征具有相同的尺度
	{
		float res;
		res = (num - mean) / std;
		return res;
	}
};