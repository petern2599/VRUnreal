// Fill out your copyright notice in the Description page of Project Settings.


#include "LeftControllerReference.h"

// Sets default values
ALeftControllerReference::ALeftControllerReference()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ALeftControllerReference::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ALeftControllerReference::TrackController(float x, float y)
{
	ControllerRefTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ControllerRefTopic->Init(rosinst->ROSIntegrationCore, TEXT("/left_controller"), TEXT("geometry_msgs/Vector3"));

	TSharedPtr<ROSMessages::geometry_msgs::Vector3> Vector3Message(new ROSMessages::geometry_msgs::Vector3(x, y, 0));
	ControllerRefTopic->Publish(Vector3Message);

}

