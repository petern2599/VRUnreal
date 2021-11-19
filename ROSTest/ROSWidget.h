// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSWidget.generated.h"

/**
 * 
 */
UCLASS()
class ROSTEST_API UROSWidget : public UUserWidget
{
	GENERATED_BODY()

		//Initializing widget
		virtual bool Initialize();

		//Creating a property of a widget and adding the button class
		UPROPERTY(meta = (BindWidget))
		class UButton* FlyOverCharacter;

public:
	UPROPERTY()
		class UTopic* FOMTopic;
public:
	UPROPERTY()
		class UTopic* DroneSelectionTopic2;
	
		//Creating reference to function for button
		UFUNCTION()
			void StartButtonClicked();
public:
	// Called every frame
	virtual void NativeTick(const FGeometry& MyGeometry, float DeltaTime);

};
