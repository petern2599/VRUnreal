// Fill out your copyright notice in the Description page of Project Settings.


#include "FOM.h"
#include "Components/Button.h"

bool isFOMTriggered = false;
extern FString DroneSelected;
FString TopicName9;

bool UFOM::Initialize()
{
	Super::Initialize();
	FlyOverCharacter->OnClicked.AddDynamic(this, &UFOM::StartButtonClicked);
	return true;
}


void UFOM::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	// Initialize a topic
	FOMTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName9 = DroneSelected + "/fom_enable";

	FOMTopic->Init(rosinst->ROSIntegrationCore, TopicName9, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	FOMTopic->Advertise();

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("FOM Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("FOM Disabled"));

	if (isFOMTriggered == false)
	{
		FOMTopic->Publish(EnableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Enabling FOM"));
		isFOMTriggered = true;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Enabling FOM")));
	}
	else
	{
		FOMTopic->Publish(DisableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Disabling FOM"));
		isFOMTriggered = false;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Disabling FOM")));
	}

}

//std::function<void(TSharedPtr<FROSBaseMsg>)> DroneSelectionCallback2 = [](TSharedPtr<FROSBaseMsg> msg) -> void
//{
//	auto Concrete0 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
//	if (Concrete0.IsValid())
//	{
//		drone_select2 = (Concrete0->_Data);
//	}
//	return;
//};
//
//void UROSWidget::NativeTick(const FGeometry& MyGeometry, float DeltaTime)
//{
//	Super::NativeTick(MyGeometry, DeltaTime);
//	// Initialize a topic
//	DroneSelectionTopic2 = NewObject<UTopic>(UTopic::StaticClass());
//	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//
//	DroneSelectionTopic2->Init(rosinst->ROSIntegrationCore, TEXT("/drone_selected"), TEXT("std_msgs/String"));
//	DroneSelectionTopic2->Subscribe(DroneSelectionCallback2);
//
//}