// Fill out your copyright notice in the Description page of Project Settings.


#include "SamplePublisher.h"

// Sets default values
ASamplePublisher::ASamplePublisher()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ASamplePublisher::BeginPlay()
{
	Super::BeginPlay();

	// Initialize a topic
	UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/chatter"), TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	ExampleTopic->Advertise();
	
	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> StringMessage(new ROSMessages::std_msgs::String("This"));
	ExampleTopic->Publish(StringMessage);
	
}

// Called every frame
void ASamplePublisher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

