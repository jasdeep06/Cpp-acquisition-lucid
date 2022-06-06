#include "stdafx.h"
#include "ArenaApi.h"
#include <algorithm>
#include <thread>
#include <string>
#include "SaveApi.h"
#include <queue>
#include <thread>
#include <condition_variable>

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "
#define ERASE_LINE "                            "


#define DELTA_TIME 167000000

#define EXPOSURE_TIME 185.0

#define PIXEL_FORMAT BGR8




std::queue<Arena::IImage*> m_queue;

bool acquisitionCompleted = false;
bool acquisitionStarted = false;

std::mutex lock;

std::condition_variable cv;


void scheduleActionCommand(Arena::ISystem* pSystem,std::vector<Arena::IDevice*>& devices,int num_images){
    for(int i = 0 ; i < num_images ; i++){
        
		auto l_start = std::chrono::high_resolution_clock::now();

		// Arena::ExecuteNode(
		// devices.at(0)->GetNodeMap(),
		// "PtpDataSetLatch");

		// // get latch
		// int64_t ptpDataSetLatchValue = Arena::GetNodeValue<int64_t>(
		// devices.at(0)->GetNodeMap(),
		// "PtpDataSetLatchValue");

		// auto l_stop = std::chrono::high_resolution_clock::now();
		// auto l_duration = std::chrono::duration_cast<std::chrono::microseconds>(l_stop - l_start);
        // 		std::cout << "Time taken by latch: "
        //     << l_duration.count() << " microseconds " << std::endl;

		// Arena::SetNodeValue<int64_t>(
        //     pSystem->GetTLSystemNodeMap(),
        //     "ActionCommandExecuteTime",
        //     ptpDataSetLatchValue + DELTA_TIME);
		Arena::SetNodeValue<int64_t>(
            pSystem->GetTLSystemNodeMap(),
            "ActionCommandExecuteTime",
            0);
        std::cout << TAB1 << "Fire action command\n";

        Arena::ExecuteNode(
            pSystem->GetTLSystemNodeMap(),
            "ActionCommandFireCommand");
		acquisitionStarted = true;
        std::this_thread::sleep_for(std::chrono::duration<float>(0.167));
    }
	acquisitionCompleted = true;
}

// void saveImage(Arena::IImage* pImage, const char* filename){


void saveImage(){

	bool localComplete = false;
	Arena::IImage* pCopy;

	while(!localComplete){

		{
				std::unique_lock<std::mutex> mu(lock);

				// wait for acquire images (producer) to notify
				cv.wait(mu, []() {return m_queue.size() > 0; });

				pCopy = m_queue.front();

				m_queue.pop();

				std::cout << TAB1 << "Size of queue " << m_queue.size() << std::endl ;


				if (m_queue.size() == 0 && acquisitionCompleted) {
					localComplete = acquisitionCompleted;
				}
		}

		std::cout << TAB1 << "Convert image to " << GetPixelFormatName(PIXEL_FORMAT) << "\n";

		auto pConverted = Arena::ImageFactory::Convert(
			pCopy,
			PIXEL_FORMAT);

		std::cout << TAB1 << "Prepare image parameters\n";

		std::string filename = "/media/vinglabs/vikz_hdd/images1/" +  std::to_string(pConverted->GetTimestamp()) +"_" + std::to_string(pConverted -> GetFrameId()) + ".jpg";

		Save::ImageParams params(
			pConverted->GetWidth(),
			pConverted->GetHeight(),
			pConverted->GetBitsPerPixel());

		std::cout << TAB1 << "Prepare image writer\n";

		Save::ImageWriter writer(
			params,
			filename.c_str());
		
		std::cout << TAB1 << "Save image\n";

		writer << pConverted->GetData();

		// destroy converted image
		Arena::ImageFactory::Destroy(pConverted);
		Arena::ImageFactory::Destroy(pCopy);

	}
}


void pollBuffer(std::vector<Arena::IDevice*>& devices,int num_images){
	
	std::cout << "entered..." << std::endl;
	while(!acquisitionStarted){
			std::cout << "waiting to start....." << std::endl;
		continue;
	}
	std::cout << "Acquisition started....."<< std::endl;
    while(1){
		auto start = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < devices.size(); i++)
        {
            Arena::IDevice* pDevice = devices.at(i);
            GenICam::gcstring deviceSerialNumber = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceSerialNumber");
            
            std::cout << TAB3 << "Timestamp: ";
			auto transfer_start = std::chrono::high_resolution_clock::now();

            // Initiate image transfer from current camera
            Arena::ExecuteNode(pDevice->GetNodeMap(), "TransferStart");
			auto wait_start = std::chrono::high_resolution_clock::now();
            Arena::IImage* pImage = pDevice->GetImage(3000);
			auto wait_end = std::chrono::high_resolution_clock::now();
			auto wait_duration = std::chrono::duration_cast<std::chrono::microseconds>(wait_end - wait_start);
        		std::cout << "Time taken by wait: "
            << wait_duration.count() << " microseconds " << pImage -> GetFrameId() <<std::endl;

            Arena::ExecuteNode(pDevice->GetNodeMap(), "TransferStop");
			auto transfer_end = std::chrono::high_resolution_clock::now();
			auto transfer_duration = std::chrono::duration_cast<std::chrono::microseconds>(transfer_end - transfer_start);
        		std::cout << "Time taken by transfer: "
            << transfer_duration.count() << " microseconds" << pImage -> GetFrameId() << std::endl;

			auto copy_start = std::chrono::high_resolution_clock::now();
            Arena::IImage* pCopy = Arena::ImageFactory::Copy(pImage);
			auto copy_end = std::chrono::high_resolution_clock::now();
			auto copy_duration = std::chrono::duration_cast<std::chrono::microseconds>(copy_end - copy_start);
        		std::cout << "Time taken by copy: "
            << copy_duration.count() << " microseconds " << pImage -> GetFrameId() <<std::endl;

            {
                // lock the thread, auto unlocks at the end of critical section
			    std::unique_lock<std::mutex> mu(lock);

			    // enqueue
			    m_queue.push(pCopy);
            
            }
			cv.notify_all();


            pDevice->RequeueBuffer(pImage);
			
        }
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Time taken by poll: "
            << duration.count() << " microseconds" << std::endl;
	}
    

}




void SynchronizeCamerasAndTriggerImage(Arena::ISystem* pSystem, std::vector<Arena::IDevice*>& devices)
{
	
	std::vector<GenICam::gcstring> exposureAutoInitials;
	std::vector<double> exposureTimeInitials;
	std::vector<bool> ptpEnableInitials;
	std::vector<GenICam::gcstring> triggerModeInitials;
	std::vector<GenICam::gcstring> triggerSourceInitials;
	std::vector<GenICam::gcstring> triggerSelectorInitials;
	std::vector<GenICam::gcstring> actionUnconditionalModeInitials;
	std::vector<int64_t> actionSelectorInitials;
	std::vector<int64_t> actionGroupKeyInitials;
	std::vector<int64_t> actionGroupMaskInitials;
	std::vector<GenICam::gcstring> transferControlModeInitials;
	std::vector<int64_t> packetSizeInitials;

	for (size_t i = 0; i < devices.size(); i++)
	{
		Arena::IDevice* pDevice = devices.at(i);
		// exposure
		exposureAutoInitials.push_back(Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureAuto"));
		exposureTimeInitials.push_back(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "ExposureTime"));
		// trigger
		triggerModeInitials.push_back(Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerMode"));
		triggerSourceInitials.push_back(Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource"));
		triggerSelectorInitials.push_back(Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSelector"));
		// action commands
		actionUnconditionalModeInitials.push_back(Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ActionUnconditionalMode"));
		actionSelectorInitials.push_back(Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "ActionSelector"));
		actionGroupKeyInitials.push_back(Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "ActionGroupKey"));
		actionGroupMaskInitials.push_back(Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "ActionGroupMask"));
		// ptp
		ptpEnableInitials.push_back(Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "PtpEnable"));
		// Transfer control
		transferControlModeInitials.push_back(Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TransferControlMode"));
		// packet size
		packetSizeInitials.push_back(Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "DeviceStreamChannelPacketSize"));
	}

	// prepare all cameras
	std::cout << TAB1 << "Setup\n";

	for (size_t i = 0; i < devices.size(); i++)
	{
		Arena::IDevice* pDevice = devices.at(i);
		GenICam::gcstring deviceSerialNumber = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceSerialNumber");

		std::cout << TAB2 << "Prepare camera " << deviceSerialNumber << "\n";

		std::cout << TAB3 << "Exposure: ";

		Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"ExposureAuto",
			"Off");

        // std::cout << Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetTLStreamNodeMap(), "StreamBufferHandlingMode") << "\n";
		Arena::SetNodeValue<double>(
			pDevice->GetNodeMap(),
			"ExposureTime",
			EXPOSURE_TIME);

		std::cout << Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "ExposureTime") << "\n";

		std::cout << TAB3 << "Trigger: ";

		Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerSelector",
			"FrameStart");

		Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerMode",
			"On");

		Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"TriggerSource",
			"Action0");

		std::cout << Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource") << "\n";


		std::cout << TAB3 << "Action commands: ";

		Arena::SetNodeValue<GenICam::gcstring>(
			pDevice->GetNodeMap(),
			"ActionUnconditionalMode",
			"On");

		Arena::SetNodeValue<int64_t>(
			pDevice->GetNodeMap(),
			"ActionSelector",
			0);

		Arena::SetNodeValue<int64_t>(
			pDevice->GetNodeMap(),
			"ActionDeviceKey",
			1);

		Arena::SetNodeValue<int64_t>(
			pDevice->GetNodeMap(),
			"ActionGroupKey",
			1);

		Arena::SetNodeValue<int64_t>(
			pDevice->GetNodeMap(),
			"ActionGroupMask",
			1);

		std::cout << "prepared\n";

		
		std::cout << TAB3 << "PTP: ";

		Arena::SetNodeValue<bool>(
			pDevice->GetNodeMap(),
			"PtpEnable",
			true);

		std::cout << (Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "PtpEnable") ? "enabled" : "disabled") << "\n";

	
		Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

		// enable stream packet resend
		Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

		
		std::cout << TAB3 << "Transfer Control: ";

		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TransferControlMode", "UserControlled");
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TransferOperationMode", "Continuous");
		Arena::ExecuteNode(pDevice->GetNodeMap(), "TransferStop");

		std::cout << Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TransferControlMode") << " - " << Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TransferOperationMode") << " - "
				<< "Transfer Stopped\n";
	}

	// prepare system
	std::cout << TAB2 << "Prepare system\n";

	
	std::cout << TAB3 << "Action commands: ";

	Arena::SetNodeValue<int64_t>(
		pSystem->GetTLSystemNodeMap(),
		"ActionCommandDeviceKey",
		1);

	Arena::SetNodeValue<int64_t>(
		pSystem->GetTLSystemNodeMap(),
		"ActionCommandGroupKey",
		1);

	Arena::SetNodeValue<int64_t>(
		pSystem->GetTLSystemNodeMap(),
		"ActionCommandGroupMask",
		1);

	Arena::SetNodeValue<int64_t>(
		pSystem->GetTLSystemNodeMap(),
		"ActionCommandTargetIP",
		0xFFFFFFFF);

	std::cout << "prepared\n";

	
	std::cout << TAB1 << "Wait for devices to negotiate. This can take up to about 40s.\n";

	std::vector<GenICam::gcstring> serials;
	int i = 0;
	do
	{
		bool masterFound = false;
		bool restartSyncCheck = false;

		// check devices
		for (size_t j = 0; j < devices.size(); j++)
		{
			Arena::IDevice* pDevice = devices.at(j);

			// get PTP status
			GenICam::gcstring ptpStatus = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PtpStatus");

			if (ptpStatus == "Master")
			{
				if (masterFound)
				{
					// Multiple masters -- ptp negotiation is not complete
					restartSyncCheck = true;
					break;
				}

				masterFound = true;
			}
			else if (ptpStatus != "Slave")
			{
				// Uncalibrated state -- ptp negotiation is not complete
				restartSyncCheck = true;
				break;
			}
		}

		// A single master was found and all remaining cameras are slaves
		if (!restartSyncCheck && masterFound)
			break;

		std::this_thread::sleep_for(std::chrono::duration<int>(1));

		// for output
		if (i % 10 == 0)
			std::cout << "\r" << ERASE_LINE << "\r" << TAB2 << std::flush;

		std::cout << "." << std::flush;

		i++;

	} while (true);

	// start stream
	std::cout << "\n"
			<< TAB1 << "Start stream\n";

	for (size_t i = 0; i < devices.size(); i++)
	{
		devices.at(i)->StartStream(15);
	}

	std::cout << TAB1 << "Set action command to " << DELTA_TIME << " nanoseconds from now\n";


	std::cout << "Starting polling thread..." << std::endl;
	std::thread polling_thread_1(pollBuffer,std::ref(devices),1000000);
	std::thread saving_thread_1(saveImage);
	std::thread saving_thread_2(saveImage);
	std::thread saving_thread_3(saveImage);
	std::thread saving_thread_4(saveImage);
	std::thread saving_thread_5(saveImage);
	std::thread saving_thread_6(saveImage);



	auto start = std::chrono::high_resolution_clock::now();
    scheduleActionCommand(pSystem,devices,1000000);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function acquire"
                << duration.count() << " microseconds" << std::endl;
	polling_thread_1.join();
	saving_thread_1.join();
	saving_thread_2.join();
	saving_thread_3.join();
	saving_thread_4.join();
	saving_thread_5.join();
	saving_thread_6.join();




	// stop stream
	std::cout << TAB1 << "Stop stream\n";

	for (size_t i = 0; i < devices.size(); i++)
	{
		devices.at(i)->StopStream();
	}

	// return nodes to their initial values
	for (size_t i = 0; i < devices.size(); i++)
	{
		Arena::IDevice* pDevice = devices.at(i);

		// packet size affects the exposure range so we restore it first
		Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "DeviceStreamChannelPacketSize", packetSizeInitials.at(i));

		// exposure
		if (exposureAutoInitials.at(i) == "Off")
		{
			Arena::SetNodeValue<double>(pDevice->GetNodeMap(), "ExposureTime", exposureTimeInitials.at(i));
		}
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureAuto", exposureAutoInitials.at(i));
		// trigger
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSelector", triggerSelectorInitials.at(i));
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource", triggerSourceInitials.at(i));
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerMode", triggerModeInitials.at(i));
		// action commands
		Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "ActionGroupMask", actionGroupMaskInitials.at(i));
		Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "ActionGroupKey", actionGroupKeyInitials.at(i));
		Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "ActionSelector", actionSelectorInitials.at(i));
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ActionUnconditionalMode", actionUnconditionalModeInitials.at(i));
		// ptp
		Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "PtpEnable", ptpEnableInitials.at(i));

		// Transfer Control
		Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TransferControlMode", transferControlModeInitials.at(i));
	}
}


int main()
{
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;

	std::cout << "Cpp_ScheduledActionCommands\n";
	// std::cout << "\nNote: The PTP auto-negotiation phase can take about 40s depending on the "
	// 		<< "initial PTP state of each device\n\n";
	// std::cout << "Example may overwrite 'ActionDeviceKey' -- proceed? ('y' to continue) ";
	// char continueExample = 'a';
	// std::cin >> continueExample;

	if (true)
	{
		try
		{
			// prepare example
			Arena::ISystem* pSystem = Arena::OpenSystem();
			pSystem->UpdateDevices(100);
			std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
			if (deviceInfos.size() < 2)
			{
				if (deviceInfos.size() == 0)
					std::cout << "\nNo camera connected. Example requires at least 2 devices\n";
				else if (deviceInfos.size() == 1)
					std::cout << "\nOnly one device connected. Example requires at least 2 devices\n";

				std::cout << "Press enter to complete\n";

				// clear input
				while (std::cin.get() != '\n')
					continue;

				std::getchar();
				return 0;
			}
			std::vector<Arena::IDevice*> devices;
			for (size_t i = 0; i < deviceInfos.size(); i++)
			{
				devices.push_back(pSystem->CreateDevice(deviceInfos.at(i)));
			}

			// run example
			std::cout << "Commence example\n\n";
			SynchronizeCamerasAndTriggerImage(pSystem, devices);
			std::cout << "\nExample complete\n";
			


			// clean up example
			for (size_t i = 0; i < devices.size(); i++)
			{
				pSystem->DestroyDevice(devices.at(i));
			}
			Arena::CloseSystem(pSystem);
		}
		catch (GenICam::GenericException& ge)
		{
			std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
			exceptionThrown = true;
		}
		catch (std::exception& ex)
		{
			std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
			exceptionThrown = true;
		}
		catch (...)
		{
			std::cout << "\nUnexpected exception thrown\n";
			exceptionThrown = true;
		}
	}

	std::cout << "Press enter to complete\n";

	// clear input
	while (std::cin.get() != '\n')
		continue;

	std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}
