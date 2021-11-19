// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROSIntegration/Public/ROSBaseServiceRequest.h"
#include "ROSIntegration/Public/ROSBaseServiceResponse.h"
#include "ServerActor.generated.h"

UCLASS()
class ROSTEST_API AServerActor : public AActor
{
    GENERATED_BODY()

public:
    // Your public code here...

protected:
    virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    // Your private code here...

    UPROPERTY()
        class UROSIntegrationGameInstance* ROSInst;

    UPROPERTY()
        class UService* AddTwoIntsServer;

    void AddTwoIntsServerCB(TSharedPtr<FROSBaseServiceRequest> Request, TSharedPtr<FROSBaseServiceResponse> Response);
};

