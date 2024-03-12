/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved. */
#pragma once

#include <ImFusion/Base/AlgorithmControllerFactory.h>
#include <ImFusion/Base/AlgorithmFactory.h>
#include <ImFusion/Base/ImFusionPlugin.h>
#include <ImFusion/Base/IoAlgorithmFactory.h>
#include "CustomROSTopicTrackingStream.h"
#include <ImFusion/Stream/CreateStreamIoAlgorithm.h>

namespace ImFusion
{

    using CustomROSTopicTrackingStreamAlgorithm = CreateStreamIoAlgorithm<CustomROSTopicTrackingStream, true, true>;

	/** \brief	Factory for Custom ROS Plugin
	 */
	class CustomROSAlgorithmFactory : public AlgorithmFactory
	{
	public:
        CustomROSAlgorithmFactory();
	};

	/** \brief	IO Factory for Custom ROS Plugin
	 */
	class CustomROSIoAlgorithmFactory : public IoAlgorithmFactory
	{
	public:
        CustomROSIoAlgorithmFactory();
	};

	/** Factory for ROS Plugin controllers
	 */
	class CustomROSControllerFactory : public AlgorithmControllerFactory
	{
	public:
        CustomROSControllerFactory();
		virtual AlgorithmController* create(Algorithm* a) const;
	};

	class CustomROSPlugin : public ImFusionPlugin
	{
	public:
        CustomROSPlugin();
		~CustomROSPlugin() override;

		const AlgorithmFactory* getAlgorithmFactory() override;

		const AlgorithmControllerFactory* getAlgorithmControllerFactory() override;

		const IoAlgorithmFactory* getIoAlgorithmFactory() override;

		std::string pluginName() const override { return "CustomImFusionROS"; }

	private:
		AlgorithmFactory* m_algFactory;
		AlgorithmControllerFactory* m_algCtrlFactory;
		IoAlgorithmFactory* m_ioAlgFactory;
	};
}

