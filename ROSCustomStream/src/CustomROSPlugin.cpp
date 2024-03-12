#include <ImFusion/Base/Utils/Containers.h>
#include <ImFusion/GUI/DefaultAlgorithmController.h>
#include "CustomROSPlugin.h"
#include <ImFusion/Stream/CreateStreamIoAlgorithm.h>

#ifndef IMFUSIONLIB_STATIC
IMFUSION_REGISTER_PLUGIN(ImFusion::CustomROSPlugin)
#endif

namespace ImFusion
{

	CustomROSPlugin::CustomROSPlugin()
	{
		//ROSPluginSettings::registerCustomSettings();
		m_algFactory = new CustomROSAlgorithmFactory;
		m_algCtrlFactory = new CustomROSControllerFactory;
		m_ioAlgFactory = new CustomROSIoAlgorithmFactory;
	}


	CustomROSPlugin::~CustomROSPlugin() = default;


	const AlgorithmFactory* CustomROSPlugin::getAlgorithmFactory()
	{
		return m_algFactory;
	}


	const AlgorithmControllerFactory* CustomROSPlugin::getAlgorithmControllerFactory()
	{
		return m_algCtrlFactory;
	}


	const IoAlgorithmFactory* CustomROSPlugin::getIoAlgorithmFactory()
	{
		return m_ioAlgFactory;
	}


	CustomROSAlgorithmFactory::CustomROSAlgorithmFactory()
		: AlgorithmFactory("ROS")
	{

	}


	CustomROSIoAlgorithmFactory::CustomROSIoAlgorithmFactory()
		: IoAlgorithmFactory("IO")
	{
        registerAlgorithm<CustomROSTopicTrackingStreamAlgorithm>("IO;CustomROSTrackingStream");
	}

	CustomROSControllerFactory::CustomROSControllerFactory() = default;

	AlgorithmController* CustomROSControllerFactory::create(Algorithm* a) const
	{
		return nullptr;
	}
}
