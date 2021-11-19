// Fill out your copyright notice in the Description page of Project Settings.


#include "ClientActor.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Service.h"
#include "rospy_tutorials/AddTwoIntsRequest.h"
#include "rospy_tutorials/AddTwoIntsResponse.h"

// Rest of your code

void AClientActor::BeginPlay()
{
    Super::BeginPlay();

    ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
    if (ROSInst)
    {
        AddTwoIntsClient = NewObject<UService>(UService::StaticClass());
        AddTwoIntsClient->Init(ROSInst->ROSIntegrationCore, TEXT("/add_two_ints"), TEXT("rospy_tutorials/AddTwoInts"));
    }
}

// For simplicity, I am putting the code to request the service in some member function.
void AClientActor::CallService()
{
    if (ROSInst)
    {
        TSharedPtr<rospy_tutorials::FAddTwoIntsRequest> Request(new rospy_tutorials::FAddTwoIntsRequest());
        Request->_a = 1.0;
        Request->_b = 2.0;
        AddTwoIntsClient->CallService(Request, std::bind(&AClientActor::AddTwoIntsResponseCB, this, std::placeholders::_1));
    }
}

void AClientActor::AddTwoIntsResponseCB(TSharedPtr<FROSBaseServiceResponse> Response)
{
    auto CastResponse = StaticCastSharedPtr<rospy_tutorials::FAddTwoIntsResponse>(Response);
    if (!CastResponse)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to cast Response to rospy_tutorials/AddTwoIntsResponse."));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("The sum is: %f"), CastResponse->_sum);
}

