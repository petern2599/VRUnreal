// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "LandingPadsNavigation.generated.h"

/**
 * 
 */
UCLASS()
class ROS_VR_API ULandingPadsNavigation : public UUserWidget
{
	GENERATED_BODY()
		//Initializing widget
		virtual bool Initialize();
public:
	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UButton* Select_Button;

	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class USpinBox* LandingPadBox;
public:
	UPROPERTY()
		class UTopic* LandingPadTopic;
	UPROPERTY()
		class UTopic* EnableLPTopic;

	UFUNCTION()
		void SelectButtonClicked();
};

