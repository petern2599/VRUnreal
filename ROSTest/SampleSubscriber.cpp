// Fill out your copyright notice in the Description page of Project Settings.

#include "SampleSubscriber.h"

// Sets default values
ASampleSubscriber::ASampleSubscriber()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ASampleSubscriber::BeginPlay()
{
	Super::BeginPlay();
	
}

// Create a std::function callback object
std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
	auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	if (Concrete.IsValid())
	{
		UE_LOG(LogTemp, Log, TEXT("Incoming string was: %s"), (*(Concrete->_Data)));
	}
	return;
};

// Called every frame
void ASampleSubscriber::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	// Initialize a topic
	class UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/chatter"), TEXT("std_msgs/String"));

	// Subscribe to the topic
	ExampleTopic->Subscribe(SubscribeCallback);
}

