// Fill out your copyright notice in the Description page of Project Settings.


#include "offboard_widget.h"
#include "Components/Button.h"

bool isOffboardTriggered = false;
extern FString DroneSelected;
FString TopicName10;

bool Uoffboard_widget::Initialize()
{
	Super::Initialize();
	Offboard->OnClicked.AddDynamic(this, &Uoffboard_widget::StartButtonClicked);
	return true;
}


void Uoffboard_widget::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	// Initialize a topic
	OffboardTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName10 = DroneSelected + "/offboard_enable";
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("%s"), *TopicName10));
	OffboardTopic->Init(rosinst->ROSIntegrationCore, TopicName10, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	OffboardTopic->Advertise();

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("Offboard Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("Offboard Disabled"));

	//Publish message
	if (isOffboardTriggered == false)
	{
		OffboardTopic->Publish(EnableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Enabling Offboard"));
		isOffboardTriggered = true;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Enabling Offboard")));
	}
	else
	{
		OffboardTopic->Publish(DisableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Disabling Offboard"));
		isOffboardTriggered = false;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Disabling Offboard")));
	}


}

//std::function<void(TSharedPtr<FROSBaseMsg>)> DroneSelectionCallback3 = [](TSharedPtr<FROSBaseMsg> msg) -> void
//{
//	auto Concrete0 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
//	if (Concrete0.IsValid())
//	{
//		drone_select2 = (Concrete0->_Data);
//	}
//	return;
//};
//
//void UOffboardWidget::NativeTick(const FGeometry& MyGeometry, float DeltaTime)
//{
//	Super::NativeTick(MyGeometry, DeltaTime);
//	// Initialize a topic
//	DroneSelectionTopic3 = NewObject<UTopic>(UTopic::StaticClass());
//	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//
//	DroneSelectionTopic3->Init(rosinst->ROSIntegrationCore, TEXT("/drone_selected"), TEXT("std_msgs/String"));
//	DroneSelectionTopic3->Subscribe(DroneSelectionCallback3);
//	
//}
