#pragma once

#include <ImFusion/Base/AlgorithmControllerFactory.h>
#include <ImFusion/Base/AlgorithmFactory.h>

namespace ImFusion {
namespace Live3DCompunding {

class Algorithm;

class PluginAlgorithmFactory : public AlgorithmFactory {
public:
  PluginAlgorithmFactory();
};

class PluginControllerFactory : public AlgorithmControllerFactory {
public:
  virtual AlgorithmController* create(ImFusion::Algorithm* a) const override;
};

}  // namespace Live3DCompunding
}  // namespace ImFusion
