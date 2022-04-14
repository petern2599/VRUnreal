// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"
#include "ROSIntegration/Public/std_msgs/MultiArrayDimension.h"
#include "ROSIntegration/Public/std_msgs/MultiArrayLayout.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "FlightHistory.generated.h"

/**
 * 
 */
UCLASS()
class ROS_VR_API UFlightHistory : public UUserWidget
{
	GENERATED_BODY()
		virtual bool Initialize();
public:
	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UButton* Select_Button;

	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class USpinBox* FlightHistoryBox;
public:
	UPROPERTY()
		class UTopic* FlightHistoryTopic;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightHistory")
		FVector current_waypoint;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightHistory")
		FVector next_waypoint;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlightHistory")
		int vector_length;

public:
	UFUNCTION()
		void SelectButtonClicked();

	UFUNCTION(BlueprintCallable, Category = "FlightHistory")
		void ShowFlightHistory(int current_index, int next_index);
};
