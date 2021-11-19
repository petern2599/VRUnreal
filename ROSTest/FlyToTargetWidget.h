// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "FlyToTargetWidget.generated.h"

/**
 * 
 */
UCLASS()
class ROSTEST_API UFlyToTargetWidget : public UUserWidget
{
	GENERATED_BODY()
		//Initializing widget
		virtual bool Initialize();

		//Creating a property of a widget and adding the button class
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
			class UEditableTextBox* XPosition;
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
			class UEditableTextBox* YPosition;
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
			class UEditableTextBox* ZPosition;
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
			class UButton* FlyToTarget;
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
			class UButton* ProjectTarget;

		//Creating reference to function for button
		UFUNCTION()
			void StartButtonClicked();

		//Creating reference to function for button
		UFUNCTION()
			void ProjectionButtonClicked();

public:
	UPROPERTY()
		class UTopic* VectorTopic;
	UPROPERTY()
		class UTopic* EnableTopic;
	UPROPERTY()
		class UTopic* DronePositionTopic;
	UPROPERTY()
		class UTopic* DroneOffsetTopic;
	UPROPERTY()
		class UTopic* DroneSelectionTopic5;
public:
		// Called every frame
		virtual void NativeTick(const FGeometry& MyGeometry, float DeltaTime);
	
};
