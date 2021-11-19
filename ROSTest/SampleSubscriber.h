// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SampleSubscriber.generated.h"

UCLASS()
class ROSTEST_API ASampleSubscriber : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASampleSubscriber();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
