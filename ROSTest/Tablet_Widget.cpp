// Fill out your copyright notice in the Description page of Project Settings.


#include "Tablet_Widget.h"
#include "Components/TextBlock.h"
#include <string>

FString drone_select;
float drone_xpos2;
float drone_ypos2;
float drone_zpos2;
float drone_xoffset2;
float drone_yoffset2;
float drone_zoffset2;
FString droneOffboardStatus;
FString droneLandStatus;
FString droneFOMStatus;
FString droneFTTStatus;
float battery_percentage;
FString armed;


bool UTablet_Widget::Initialize()
{
	Super::Initialize();
	
	return true;
}

// Create a std::function callback object

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneSelectionCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete0 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete0.IsValid())
	{
		drone_select = (Concrete0->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneCallback2 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (Concrete.IsValid())
	{
		drone_xpos2 = (Concrete->pose.position.y);
		drone_ypos2 = (Concrete->pose.position.x);
		drone_zpos2 = (Concrete->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback2 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete2 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (Concrete2.IsValid())
	{
		drone_xoffset2 = (Concrete2->x);
		drone_yoffset2 = (Concrete2->y);
		drone_zoffset2 = (Concrete2->z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffboardCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete3 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete3.IsValid())
	{
		droneOffboardStatus = (Concrete3->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneLandCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete4 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete4.IsValid())
	{
		droneLandStatus = (Concrete4->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneFOMCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete5 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete5.IsValid())
	{
		droneFOMStatus = (Concrete5->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneFTTCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete6 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete6.IsValid())
	{
		droneFTTStatus = (Concrete6->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> BatteryPercentageCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete7 = StaticCastSharedPtr<ROSMessages::std_msgs::Float32>(msg);
	if (Concrete7.IsValid())
	{
		battery_percentage = (Concrete7->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> ArmedCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete8 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete8.IsValid())
	{
		armed = (Concrete8->_Data);
	}
	return;
};

void UTablet_Widget::NativeTick(const FGeometry& MyGeometry, float DeltaTime)
{
	Super::NativeTick(MyGeometry, DeltaTime);
	// Initialize a topic
	DroneSelectionTopic = NewObject<UTopic>(UTopic::StaticClass());
	DronePositionTopic2 = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic2 = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffboardTopic = NewObject<UTopic>(UTopic::StaticClass());
	DroneFOMTopic = NewObject<UTopic>(UTopic::StaticClass());
	DroneFTTTopic = NewObject<UTopic>(UTopic::StaticClass());
	DroneLandTopic = NewObject<UTopic>(UTopic::StaticClass());
	BatteryPercentageTopic = NewObject<UTopic>(UTopic::StaticClass());
	ArmedTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	
	DroneSelectionTopic->Init(rosinst->ROSIntegrationCore, TEXT("/drone_selected"), TEXT("std_msgs/String"));
	DroneSelectionTopic->Subscribe(DroneSelectionCallback);

	FString TopicName1 = drone_select + "/mavros/local_position/pose";
	FString TopicName2 = drone_select + "/drone_offset";
	FString TopicName3 = drone_select + "/offboard_enable";
	FString TopicName4 = drone_select + "/land_enable";
	FString TopicName5 = drone_select + "/fom_enable";
	FString TopicName6 = drone_select + "/ftt_enable";
	FString TopicName7 = drone_select + "/battery_percentage";
	FString TopicName8 = drone_select + "/armed_status";

	DronePositionTopic2->Init(rosinst->ROSIntegrationCore, TopicName1, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic2->Init(rosinst->ROSIntegrationCore, TopicName2, TEXT("geometry_msgs/Vector3"));
	DroneOffboardTopic->Init(rosinst->ROSIntegrationCore, TopicName3, TEXT("std_msgs/String"));
	DroneLandTopic->Init(rosinst->ROSIntegrationCore, TopicName4, TEXT("std_msgs/String"));
	DroneFOMTopic->Init(rosinst->ROSIntegrationCore, TopicName5, TEXT("std_msgs/String"));
	DroneFTTTopic->Init(rosinst->ROSIntegrationCore, TopicName6, TEXT("std_msgs/String"));
	BatteryPercentageTopic->Init(rosinst->ROSIntegrationCore, TopicName7, TEXT("std_msgs/Float32"));
	ArmedTopic->Init(rosinst->ROSIntegrationCore, TopicName8, TEXT("std_msgs/String"));

	// Subscribe to the topic
	
	DronePositionTopic2->Subscribe(DroneCallback2);
	DroneOffsetTopic2->Subscribe(DroneOffsetCallback2);
	DroneOffboardTopic->Subscribe(DroneOffboardCallback);
	DroneLandTopic->Subscribe(DroneLandCallback);
	DroneFOMTopic->Subscribe(DroneFOMCallback);
	DroneFTTTopic->Subscribe(DroneFTTCallback);
	BatteryPercentageTopic->Subscribe(BatteryPercentageCallback);
	ArmedTopic->Subscribe(ArmedCallback);

	
	float xposUE = (drone_xpos2 - drone_xoffset2) * 100;
	float yposUE = (drone_ypos2 - drone_yoffset2) * 100;
	float zposUE = (drone_zpos2 + drone_zoffset2) * 100;

	float xposMAVROS = (drone_ypos2 - drone_yoffset2);
	float yposMAVROS = (drone_xpos2 - drone_xoffset2);
	float zposMAVROS = (drone_zpos2 + drone_zoffset2);

	FString float_str = FString::SanitizeFloat(battery_percentage * 100);
	FString battery_percentage_str = float_str + "%";


	if (UE_XPosition)
	{
		UE_XPosition->SetText(FText::AsNumber(xposUE));
	}
	
	if (UE_YPosition)
	{
		UE_YPosition->SetText(FText::AsNumber(yposUE));
	}
	if (UE_ZPosition)
	{
		UE_ZPosition->SetText(FText::AsNumber(zposUE));
	}

	if (MAVROS_XPosition)
	{
		MAVROS_XPosition->SetText(FText::AsNumber(xposMAVROS));
	}

	if (MAVROS_YPosition)
	{
		MAVROS_YPosition->SetText(FText::AsNumber(yposMAVROS));
	}

	if (MAVROS_ZPosition)
	{
		MAVROS_ZPosition->SetText(FText::AsNumber(zposMAVROS));
	}
	
	if (Drone_Offboard_Status)
	{
		Drone_Offboard_Status->SetText(FText::FromString(droneOffboardStatus));
	}
	
	if (FOM_Status)
	{
		FOM_Status->SetText(FText::FromString(droneFOMStatus));
	}

	if (FTT_Status)
	{
		FTT_Status->SetText(FText::FromString(droneFTTStatus));
	}

	if (Drone_Land_Status)
	{
		Drone_Land_Status->SetText(FText::FromString(droneLandStatus));
	}

	if (Battery_Percent)
	{
		
		Battery_Percent->SetText(FText::FromString(battery_percentage_str));
	}

	if (Armed)
	{

		Armed->SetText(FText::FromString(armed));
	}

	if (SelectedDrone)
	{
		SelectedDrone->SetText(FText::FromString(drone_select));
	}

}

