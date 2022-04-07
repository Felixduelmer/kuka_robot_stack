#include <iostream>
#include "imf_publisher_plugin/algorithm.h"
#include <ImFusion/Stream/FakeTrackingStream.h>
#include <ImFusion/Stream/FakeImageStream.h>
#include <ImFusion/Base/ImFusionFile.h>
#include <string>
#include <chrono>
#include <ImFusion/GUI/MainWindowBase.h>
#include <thread>
#include <ImFusion/US/GlSweepCompounding.h>        // for GlSweepCompounding


int main(int argc, char* argv[]) {


    ImFusion::ImFusionFile file("/home/robotics-verse/Desktop/testSweep.imf");
    file.open(0);

    ImFusion::DataList dataList;
    file.load(dataList);

    ImFusion::UltrasoundSweep* usSweep = static_cast<ImFusion::UltrasoundSweep*>(dataList.getItem(0));

     std::cout << "us sweep length: " << usSweep->size() << std::endl;
     ImFusion::Selection frame_selection;
//     frame_selection.setAll(usSweep->size());
//     usSweep->setSelection(frame_selection);

//    ImFusion::GlSweepCompounding compounder(*usSweep);

//    // Set the options of the compounder.
//    compounder.setSpacing(0);
//    compounder.setHideSweep(false);
//    compounder.setMode(1);
//    compounder.setVerbose(true);
//    compounder.compute();
//    ImFusion::OwningDataList result;
//    result = compounder.takeOutput();


//    ImFusion::FakeTrackingStream fake_tracking_stream{};
//    ImFusion::FakeImageStream fake_image_stream{1800, 1200, 1, 3};
//    ImFusion::ImfPublisher::Sweeper sweeper{&fake_tracking_stream, &fake_image_stream, "Test Sweeper"};

//    sweeper.addSweep(usSweep);
//    sweeper.compound();
//    fake_image_stream.open();
//    fake_image_stream.start();
//    fake_tracking_stream.open();
//    fake_tracking_stream.start();
//    if(!fake_image_stream.isRunning() || !fake_tracking_stream.isRunning()){
//        std::cout << "streams are not running" << std::endl;
//    }

//    sweeper.recordSweep();
//    std::this_thread::sleep_for(std::chrono::seconds(2));
//    sweeper.saveSweep();
//    sweeper.compound();


    std::cout << "Programm finished" << std::endl;

}

