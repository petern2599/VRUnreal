// Fill out your copyright notice in the Description page of Project Settings.


#include "FlightHistory.h"
#include "Components/SpinBox.h"
#include "Components/Button.h"
#include "DrawDebugHelpers.h"
#include <vector>

std::vector<float> vec_x;
std::vector<float> vec_y;
std::vector<float> vec_z;


bool UFlightHistory::Initialize()
{
	Super::Initialize();
	Select_Button->OnClicked.AddDynamic(this, &UFlightHistory::SelectButtonClicked);
	return true;
}

std::function<void(TSharedPtr<FROSBaseMsg>)> FlightHistory_Callback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto array_msg = StaticCastSharedPtr<ROSMessages::std_msgs::Float32MultiArray>(msg);
	if(array_msg.IsValid())
	{
		for (uint32 x{ 0 }; x < array_msg -> layout.dim[0].size; x++) {
			vec_x.push_back(array_msg->data[x]);
			
		}
		for (uint32 y{ array_msg->layout.dim[0].size }; y < array_msg->layout.dim[0].size + array_msg->layout.dim[1].size; y++) {
			vec_y.push_back(array_msg->data[y]);
			
		}
		for (uint32 z{ array_msg->layout.dim[0].size + array_msg->layout.dim[1].size }; z < array_msg->layout.dim[0].size + array_msg->layout.dim[1].size + array_msg->layout.dim[2].size; z++) {
			vec_z.push_back(array_msg->data[z]);
			
		}
	}
	return;
};

void UFlightHistory::SelectButtonClicked()
{
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
	// Initialize a topic
	FlightHistoryTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());


	int FlightHistoryInt = FlightHistoryBox->GetValue();
	FString FlightHistoryStr = FString::FromInt(FlightHistoryInt);
	FString TopicName = "/uav" + FlightHistoryStr + "/flight_history";
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, TopicName);
	FlightHistoryTopic->Init(rosinst->ROSIntegrationCore, TopicName, TEXT("std_msgs/Float32MultiArray"));

	// (Optional) Advertise the topic
	FlightHistoryTopic->Subscribe(FlightHistory_Callback);

	vector_length = vec_x.size();

}

void UFlightHistory::ShowFlightHistory(int current_index, int next_index)
{

	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
	


	if (vec_x.size() != 0 && vec_y.size() != 0 && vec_z.size() != 0) {
		current_waypoint = FVector(vec_x[current_index] * 100, vec_y[current_index] * 100, vec_z[current_index] * 100);
		DrawDebugPoint(GetWorld(), current_waypoint, 20, FColor(255, 0, 255), false, 3.0f);

		next_waypoint = FVector(vec_x[next_index] * 100, vec_y[next_index] * 100, vec_z[next_index] * 100);
		DrawDebugPoint(GetWorld(), next_waypoint, 20, FColor(124, 252, 0), false, 3.0f);

		DrawDebugDirectionalArrow(GetWorld(), current_waypoint, next_waypoint, 200.f, FColor(255, 0, 255), false, 3.0f, 3.0f);
		/*if (vec_x.size() > 2 && vec_y.size() > 2 && vec_z.size() > 2) {
			for (int curr{ 2 }; curr < vec_x.size(); curr++) {
				current_waypoint = FVector(vec_x[curr - 1] * 100, vec_y[curr - 1] * 100, vec_z[curr - 1] * 100);
				DrawDebugPoint(GetWorld(), current_waypoint, 20, FColor(255, 0, 255), false, 3.0f);

				next_waypoint = FVector(vec_x[curr] * 100, vec_y[curr] * 100, vec_z[curr] * 100);
				DrawDebugPoint(GetWorld(), next_waypoint, 20, FColor(124, 252, 0), false, 3.0f);

				DrawDebugDirectionalArrow(GetWorld(), current_waypoint, next_waypoint, 200.f, FColor(255, 0, 255), false, 3.0f, 3.0f);
			}
		}*/
	}
	else {
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("No Waypoints to Project")));
	}
}
