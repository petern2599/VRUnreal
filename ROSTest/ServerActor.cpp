// Fill out your copyright notice in the Description page of Project Settings.

#include "ServerActor.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Service.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"
#include "rospy_tutorials/AddTwoIntsResponse.h"

// Rest of your code

void AServerActor::BeginPlay()
{
    Super::BeginPlay();

    ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
    if (ROSInst)
    {
        AddTwoIntsServer = NewObject<UService>(UService::StaticClass());
        AddTwoIntsServer->Init(ROSInst->ROSIntegrationCore, TEXT("/add_two_ints"), TEXT("rospy_tutorials/AddTwoInts"));

        // The second param indicates if we wish to execute the callback in the game thread. 
        // I chose false, to be consistent with how topic callbacks work.
        AddTwoIntsServer->Advertise(std::bind(&AServerActor::AddTwoIntsServerCB, this, std::placeholders::_1, std::placeholders::_2), false);
    }
}

void AServerActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    AddTwoIntsServer->Unadvertise(); // If you do not unadvertise/unsubscribe in EndPlay, weird behavior may arise
}

void AServerActor::AddTwoIntsServerCB(TSharedPtr<FROSBaseServiceRequest> Request, TSharedPtr<FROSBaseServiceResponse> Response)
{
    auto CastRequest = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsRequest>(Request);
    auto CastResponse = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsResponse>(Response);
    if (!CastRequest)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to cast Request to rospy_tutorials/AddTwoIntsRequest."));
        return;
    }
    if (!CastResponse)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to cast Response to rospy_tutorials/AddTwoIntsResponse."));
        return;
    }
    CastResponse->_sum = CastRequest->_a + CastRequest->_b;
}
