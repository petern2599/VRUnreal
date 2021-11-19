// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"
#include "ClientActor.generated.h"

UCLASS()
class ROSTEST_API AClientActor : public AActor
{
    GENERATED_BODY()

public:
    // Your public code here...

protected:
    virtual void BeginPlay() override;

private:
    // Your private code here...

    UPROPERTY()
        class UROSIntegrationGameInstance* ROSInst;

    UPROPERTY()
        class UService* AddTwoIntsClient;

    void CallService();
    void AddTwoIntsResponseCB(TSharedPtr<FROSBaseServiceResponse> Response);
};

