// Fill out your copyright notice in the Description page of Project Settings.


#include "LandWidget.h"
#include "Components/Button.h"

bool isLandTriggered = false;
FString drone_select4;
FString TopicName11;

bool ULandWidget::Initialize()
{
	Super::Initialize();
	Land->OnClicked.AddDynamic(this, &ULandWidget::StartButtonClicked);
	return true;
}


void ULandWidget::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	// Initialize a topic
	LandTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName11 = drone_select4 + "/land_enable";
	UE_LOG(LogTemp, Warning, TEXT("%s"), *TopicName11);

	LandTopic->Init(rosinst->ROSIntegrationCore, TopicName11, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	LandTopic->Advertise();

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("Land Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("Land Disabled"));

	//Publish message
	if (isLandTriggered == false)
	{
		LandTopic->Publish(EnableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Landing Vehicle"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Land Enabled")));
		isLandTriggered = true;
	}
	else
	{
		LandTopic->Publish(DisableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Disabling Land"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Land Disabled")));
		isLandTriggered = false;
	}

}

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneSelectionCallback4 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete0 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete0.IsValid())
	{
		drone_select4 = (Concrete0->_Data);
	}
	return;
};

void ULandWidget::NativeTick(const FGeometry& MyGeometry, float DeltaTime)
{
	Super::NativeTick(MyGeometry, DeltaTime);
	// Initialize a topic
	DroneSelectionTopic4 = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	DroneSelectionTopic4->Init(rosinst->ROSIntegrationCore, TEXT("/drone_selected"), TEXT("std_msgs/String"));
	DroneSelectionTopic4->Subscribe(DroneSelectionCallback4);

}
