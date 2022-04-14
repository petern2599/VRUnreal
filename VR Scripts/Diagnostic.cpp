// Fill out your copyright notice in the Description page of Project Settings.


#include "Diagnostic.h"
#include "Components/TextBlock.h"
#include <string>

extern float drone0_x;
extern float drone0_y;
extern float drone0_z;

extern float drone1_x;
extern float drone1_y;
extern float drone1_z;

extern float drone2_x;
extern float drone2_y;
extern float drone2_z;

float xposUE;
float yposUE;
float zposUE;

float xposMAVROS;
float yposMAVROS;
float zposMAVROS;

float battery_percentage;
FString armed;
//FString state;

bool UDiagnostic::Initialize()
{
	Super::Initialize();

	return true;
}

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

//std::function<void(TSharedPtr<FROSBaseMsg>)> StateCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
//{
//	auto Concrete9 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
//	if (Concrete9.IsValid())
//	{
//		state = (Concrete9->_Data);
//	}
//	return;
//};

void UDiagnostic::GetAllDroneStatus(int drone_number)
{
	//Super::NativeTick(MyGeometry, DeltaTime);
	// Initialize a topic
	BatteryPercentageTopic = NewObject<UTopic>(UTopic::StaticClass());
	ArmedTopic = NewObject<UTopic>(UTopic::StaticClass());
	/*StateTopic = NewObject<UTopic>(UTopic::StaticClass());*/

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	FString drone_number_str = FString::FromInt(drone_number);

	
	FString BatteryTopicName = "/uav" + drone_number_str + "/battery_percentage";
	FString ArmedTopicName = "/uav" + drone_number_str + "/armed_status";

	BatteryPercentageTopic->Init(rosinst->ROSIntegrationCore, BatteryTopicName, TEXT("std_msgs/Float32"));
	ArmedTopic->Init(rosinst->ROSIntegrationCore, ArmedTopicName, TEXT("std_msgs/String"));

	// Subscribe to the topic

	BatteryPercentageTopic->Subscribe(BatteryPercentageCallback);
	ArmedTopic->Subscribe(ArmedCallback);

	switch (drone_number) {
	case 0:
		//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Working")));
		xposUE = (drone0_x) * 100;
		yposUE = (drone0_y) * 100;
		zposUE = (drone0_z) * 100;

		xposMAVROS = (drone0_y);
		yposMAVROS = (drone0_x);
		zposMAVROS = (drone0_z);
		break;
	case 1:
		xposUE = (drone1_x) * 100;
		yposUE = (drone1_y) * 100;
		zposUE = (drone1_z) * 100;

		xposMAVROS = (drone1_y);
		yposMAVROS = (drone1_x);
		zposMAVROS = (drone1_z);
		break;
	case 2:
		xposUE = (drone2_x) * 100;
		yposUE = (drone2_y) * 100;
		zposUE = (drone2_z) * 100;

		xposMAVROS = (drone2_y);
		yposMAVROS = (drone2_x);
		zposMAVROS = (drone2_z);
		break;
	}
	

	FString float_str = FString::SanitizeFloat(battery_percentage * 100);
	FString battery_percentage_str = float_str + "%";


	if (UE4_XPosition)
	{
		UE4_XPosition->SetText(FText::AsNumber(xposUE));
	}

	if (UE4_YPosition)
	{
		UE4_YPosition->SetText(FText::AsNumber(yposUE));
	}
	if (UE4_ZPosition)
	{
		UE4_ZPosition->SetText(FText::AsNumber(zposUE));
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


	if (Battery_Percent)
	{

		Battery_Percent->SetText(FText::FromString(battery_percentage_str));
	}

	if (Armed)
	{

		Armed->SetText(FText::FromString(armed));
	}

	/*if (State)
	{

		State->SetText(FText::FromString(state));
	}*/
}