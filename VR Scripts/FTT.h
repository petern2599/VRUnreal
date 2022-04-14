// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"
#include "ROSIntegration/Public/std_msgs/MultiArrayDimension.h"
#include "ROSIntegration/Public/std_msgs/MultiArrayLayout.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "FTT.generated.h"

/**
 * 
 */
UCLASS()
class ROS_VR_API UFTT : public UUserWidget
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
public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UButton* EnterWaypoint;
public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UButton* ClearWaypoint;
public:
	//UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
	//	class UButton* FlightHistory;

	//Creating reference to function for button
	UFUNCTION()
		void StartButtonClicked();

	//Creating reference to function for button
	UFUNCTION()
		void ProjectionButtonClicked();

	//Creating reference to function for button
	UFUNCTION()
		void EnterButtonClicked();

	//Creating reference to function for button
	UFUNCTION()
		void ClearButtonClicked();

	////Creating reference to function for button
	//UFUNCTION()
	//	void HistoryButtonClicked();

public:
	UPROPERTY()
		class UTopic* MultiArrayTopic;
	UPROPERTY()
		class UTopic* EnableTopic;


};
