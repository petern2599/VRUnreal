// Fill out your copyright notice in the Description page of Project Settings.


#include "FTT.h"
#include "Components/EditableTextBox.h"
#include "Components/Button.h"
#include "DrawDebugHelpers.h"
#include <vector>

bool isFTTTriggered = false;
extern float drone_xpos;
extern float drone_ypos;
extern float drone_zpos;
extern float drone_xoffset;
extern float drone_yoffset;
extern float drone_zoffset;
extern FString DroneSelected;
extern int DroneNumberInt;

FString TopicName12;
FString TopicName13;
FString TopicName14;
FString TopicName15;

std::vector<float> vecX;
std::vector<float> vecY;
std::vector<float> vecZ;

//std::vector<std::vector<float>> flight_history;
//std::vector<float> flight_historyX;
//std::vector<float> flight_historyY;
//std::vector<float> flight_historyZ;

bool UFTT::Initialize()
{
	Super::Initialize();
	FlyToTarget->OnClicked.AddDynamic(this, &UFTT::StartButtonClicked);
	ProjectTarget->OnClicked.AddDynamic(this, &UFTT::ProjectionButtonClicked);
	EnterWaypoint->OnClicked.AddDynamic(this, &UFTT::EnterButtonClicked);
	ClearWaypoint->OnClicked.AddDynamic(this, &UFTT::ClearButtonClicked);
	/*FlightHistory->OnClicked.AddDynamic(this, &UFlyToTargetWidget::HistoryButtonClicked);*/
	return true;
}

void UFTT::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	// Initialize a topic
	MultiArrayTopic = NewObject<UTopic>(UTopic::StaticClass());
	EnableTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName12 = DroneSelected + "/target_position";
	TopicName13 = DroneSelected + "/ftt_enable";

	MultiArrayTopic->Init(rosinst->ROSIntegrationCore, TopicName12, TEXT("std_msgs/Float32MultiArray"));
	EnableTopic->Init(rosinst->ROSIntegrationCore, TopicName13, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	MultiArrayTopic->Advertise();
	EnableTopic->Advertise();


	/*(*MultiArrayMessage).data.Add(1.0);
	(*MultiArrayMessage).data.Add(2.0);
	(*MultiArrayMessage).data.Add(3.0);*/

	//TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> MultiArrayMessage(new ROSMessages::geometry_msgs::Vector3(xPos_float, yPos_float, zPos_float));
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("FTT Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("FTT Disabled"));


	//Publish message
	if (isFTTTriggered == false)
	{
		TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> MultiArrayMessage(new ROSMessages::std_msgs::Float32MultiArray);

		ROSMessages::std_msgs::MultiArrayDimension dim_x;
		dim_x.size = vecX.size();
		dim_x.stride = 1;
		dim_x.label = "X";
		(*MultiArrayMessage).layout.dim.Add(dim_x);

		ROSMessages::std_msgs::MultiArrayDimension dim_y;
		dim_y.size = vecY.size();
		dim_y.stride = 1;
		dim_y.label = "Y";
		(*MultiArrayMessage).layout.dim.Add(dim_y);

		ROSMessages::std_msgs::MultiArrayDimension dim_z;
		dim_z.size = vecZ.size();
		dim_z.stride = 1;
		dim_z.label = "Z";
		(*MultiArrayMessage).layout.dim.Add(dim_z);
		(*MultiArrayMessage).layout.data_offset = 0;

		for (int i{ 0 }; i < vecX.size(); i++) {
			(*MultiArrayMessage).data.Add(vecX.at(i));
		}

		for (int j{ 0 }; j < vecY.size(); j++) {

			(*MultiArrayMessage).data.Add(vecY.at(j));
		}

		for (int k{ 0 }; k < vecZ.size(); k++) {
			(*MultiArrayMessage).data.Add(vecZ.at(k));
		}

		/*for (int x{ 0 }; x < vecX.size(); x++) {
			flight_historyX.push_back(vecX.at(x));
		}
		for (int y{ 0 }; y < vecY.size(); y++) {
			flight_historyY.push_back(vecY.at(y));
		}
		for (int z{ 0 }; z < vecZ.size(); z++) {
			flight_historyZ.push_back(vecZ.at(z));
		}*/

		MultiArrayTopic->Publish(MultiArrayMessage);
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
		vecX.clear();
		vecY.clear();
		vecZ.clear();
	}


}


void UFTT::ProjectionButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
	FVector current_waypoint;
	FVector next_waypoint;


	if (vecX.size() != 0 && vecY.size() != 0 && vecZ.size() != 0) {
		current_waypoint = FVector((drone_ypos + drone_yoffset) * 100, (drone_xpos + drone_xoffset) * 100, (drone_zpos + drone_zoffset) * 100);
		DrawDebugPoint(GetWorld(), current_waypoint, 20, FColor(255, 0, 255), true, 3.0f);

		next_waypoint = FVector(vecX[0] * 100, vecY[0] * 100, vecZ[0] * 100);
		DrawDebugPoint(GetWorld(), next_waypoint, 20, FColor(124, 252, 0), true, 3.0f);

		DrawDebugDirectionalArrow(GetWorld(), current_waypoint, next_waypoint, 200.f, FColor(255, 0, 255), true, 3.0f, 3.0f);
		for (int curr{ 1 }; curr < vecX.size(); curr++) {
			current_waypoint = FVector(vecX[curr - 1] * 100, vecY[curr - 1] * 100, vecZ[curr - 1] * 100);
			DrawDebugPoint(GetWorld(), current_waypoint, 20, FColor(255, 0, 255), true, 3.0f);

			next_waypoint = FVector(vecX[curr] * 100, vecY[curr] * 100, vecZ[curr] * 100);
			DrawDebugPoint(GetWorld(), next_waypoint, 20, FColor(124, 252, 0), true, 3.0f);

			DrawDebugDirectionalArrow(GetWorld(), current_waypoint, next_waypoint, 200.f, FColor(255, 0, 255), true, 3.0f, 3.0f);
		}
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("No Waypoints to Project")));
	}

}

void UFTT::EnterButtonClicked()
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


	vecX.push_back(xPos_float);
	vecY.push_back(yPos_float);
	vecZ.push_back(zPos_float);
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Waypoint Added")));
}

void UFTT::ClearButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
	if (vecX.size() != 0 && vecY.size() != 0 && vecZ.size() != 0) {
		vecX.pop_back();
		vecY.pop_back();
		vecZ.pop_back();
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("No More Waypoints to Clear")));
	}

}

//void UFlyToTargetWidget::HistoryButtonClicked()
//{
//	//Check if button works
//	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
//	FVector current_waypoint;
//	FVector next_waypoint;
//
//
//	if (flight_historyX.size() != 0 && flight_historyY.size() != 0 && flight_historyZ.size() != 0) {
//		current_waypoint = FVector(flight_historyX[0] * 100, flight_historyY[0] * 100, flight_historyZ[0] * 100);
//		DrawDebugPoint(GetWorld(), current_waypoint, 20, FColor(255, 0, 255), false, 3.0f);
//
//		next_waypoint = FVector(flight_historyX[1] * 100, flight_historyY[1] * 100, flight_historyZ[1] * 100);
//		DrawDebugPoint(GetWorld(), next_waypoint, 20, FColor(124, 252, 0), false, 3.0f);
//
//		DrawDebugDirectionalArrow(GetWorld(), current_waypoint, next_waypoint, 200.f, FColor(255, 0, 255), false, 3.0f, 3.0f);
//		if (flight_historyX.size() > 2 && flight_historyY.size() > 2 && flight_historyZ.size() > 2) {
//			for (int curr{ 2 }; curr < vecX.size(); curr++) {
//				current_waypoint = FVector(flight_historyX[curr-1] * 100, flight_historyY[curr-1] * 100, flight_historyZ[curr-1] * 100);
//				DrawDebugPoint(GetWorld(), current_waypoint, 20, FColor(255, 0, 255), false, 3.0f);
//
//				next_waypoint = FVector(flight_historyX[curr] * 100, flight_historyY[curr] * 100, flight_historyZ[curr] * 100);
//				DrawDebugPoint(GetWorld(), next_waypoint, 20, FColor(124, 252, 0), false, 3.0f);
//
//				DrawDebugDirectionalArrow(GetWorld(), current_waypoint, next_waypoint, 200.f, FColor(255, 0, 255), false, 3.0f, 3.0f);
//			}
//		}
//	}
//	else {
//		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("No Waypoints to Project")));
//	}
//
//}
