// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "SamplePublisher.generated.h"

UCLASS()
class ROSTEST_API ASamplePublisher : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASamplePublisher();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
