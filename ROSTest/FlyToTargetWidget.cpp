// Fill out your copyright notice in the Description page of Project Settings.


#include "FlyToTargetWidget.h"
#include "Components/EditableTextBox.h"
#include "Components/Button.h"
#include "DrawDebugHelpers.h"

bool isFTTTriggered = false;
float drone_xpos;
float drone_ypos;
float drone_zpos;
float drone_xoffset;
float drone_yoffset;
float drone_zoffset;
FString drone_select5;
FString TopicName12;
FString TopicName13;
FString TopicName14;
FString TopicName15;

bool UFlyToTargetWidget::Initialize()
{
	Super::Initialize();
	FlyToTarget->OnClicked.AddDynamic(this, &UFlyToTargetWidget::StartButtonClicked);
	ProjectTarget->OnClicked.AddDynamic(this, &UFlyToTargetWidget::ProjectionButtonClicked);
	return true;
}

void UFlyToTargetWidget::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
	
	FText xPos = XPosition->GetText();
	FString xPos_String = xPos.ToString();
	float xPos_float = FCString::Atof(*xPos_String);

	FText yPos = YPosition->GetText();
	FString yPos_String = yPos.ToString();
	float yPos_float = FCString::Atof(*yPos_String);

	FText zPos = ZPosition->GetText();
	FString zPos_String = zPos.ToString();
	float zPos_float = FCString::Atof(*zPos_String);

	// Initialize a topic
	VectorTopic = NewObject<UTopic>(UTopic::StaticClass());
	EnableTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName12 = drone_select5 + "/target_position";
	TopicName13 = drone_select5 + "/ftt_enable";

	VectorTopic->Init(rosinst->ROSIntegrationCore, TopicName12, TEXT("geometry_msgs/Vector3"));
	EnableTopic->Init(rosinst->ROSIntegrationCore, TopicName13, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	VectorTopic->Advertise();
	EnableTopic->Advertise();

	// Publish a vector to the topic
	TSharedPtr<ROSMessages::geometry_msgs::Vector3> VectorMessage(new ROSMessages::geometry_msgs::Vector3(xPos_float, yPos_float, zPos_float));
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("FTT Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("FTT Disabled"));

	//Publish message
	if (isFTTTriggered == false)
	{
		VectorTopic->Publish(VectorMessage);
		EnableTopic->Publish(EnableMessage);
		isFTTTriggered = true;
		UE_LOG(LogTemp, Warning, TEXT("Sending Target Position"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Sending Target Position")));
	}
	else
	{
		EnableTopic->Publish(DisableMessage);
		isFTTTriggered = false;
		UE_LOG(LogTemp, Warning, TEXT("Disabling FTT"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Disabling FTT")));
	}

	
}

// Create a std::function callback object

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneSelectionCallback5 = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete0 = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete0.IsValid())
	{
		drone_select5 = (Concrete0->_Data);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete = StaticCastSharedPtr<ROSMessages::geometry_msgs::PoseStamped>(msg);
	if (Concrete.IsValid())
	{
		drone_xpos = (Concrete->pose.position.x);
		drone_ypos = (Concrete->pose.position.y);
		drone_zpos = (Concrete->pose.position.z);
	}
	return;
};

std::function<void(TSharedPtr<FROSBaseMsg>)> DroneOffsetCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete2 = StaticCastSharedPtr<ROSMessages::geometry_msgs::Vector3>(msg);
	if (Concrete2.IsValid())
	{
		drone_xoffset = (Concrete2->x);
		drone_yoffset = (Concrete2->y);
		drone_zoffset = (Concrete2->z);
	}
	return;
};

// Called every frame
void UFlyToTargetWidget::NativeTick(const FGeometry& MyGeometry, float DeltaTime)
{
	Super::NativeTick(MyGeometry,DeltaTime);
	// Initialize a topic
	DroneSelectionTopic5 = NewObject<UTopic>(UTopic::StaticClass());
	DronePositionTopic = NewObject<UTopic>(UTopic::StaticClass());
	DroneOffsetTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	DroneSelectionTopic5->Init(rosinst->ROSIntegrationCore, TEXT("/drone_selected"), TEXT("std_msgs/String"));
	DroneSelectionTopic5->Subscribe(DroneSelectionCallback5);

	TopicName14 = drone_select5 + "/mavros/local_position/pose";
	TopicName15 = drone_select5 + "/drone_offset";

	DronePositionTopic->Init(rosinst->ROSIntegrationCore, TopicName14, TEXT("geometry_msgs/PoseStamped"));
	DroneOffsetTopic->Init(rosinst->ROSIntegrationCore, TopicName15, TEXT("geometry_msgs/Vector3"));
	

	// Subscribe to the topic
	DronePositionTopic->Subscribe(DroneCallback);
	DroneOffsetTopic->Subscribe(DroneOffsetCallback);
}

void UFlyToTargetWidget::ProjectionButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	FText xPos = XPosition->GetText();
	FString xPos_String = xPos.ToString();
	float xPos_float = FCString::Atof(*xPos_String);

	FText yPos = YPosition->GetText();
	FString yPos_String = yPos.ToString();
	float yPos_float = FCString::Atof(*yPos_String);

	FText zPos = ZPosition->GetText();
	FString zPos_String = zPos.ToString();
	float zPos_float = FCString::Atof(*zPos_String);


	FVector droneLocation = FVector((drone_ypos - drone_xoffset) * 100, (drone_xpos - drone_yoffset) * 100, (drone_zpos + drone_zoffset) * 100);
	DrawDebugPoint(GetWorld(), droneLocation, 20, FColor(255, 0, 255), false, 3.0f);

	FVector targetLocation = FVector(xPos_float * 100, yPos_float * 100, zPos_float * 100);
	DrawDebugPoint(GetWorld(), targetLocation, 20, FColor(124,252, 0), false, 3.0f);

	DrawDebugDirectionalArrow(GetWorld(), droneLocation, targetLocation, 200.f, FColor(255, 0, 255), false, 3.0f, 3.0f);
}