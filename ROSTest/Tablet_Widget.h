// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/std_msgs/Float32.h"
#include "ROSIntegration/Public/geometry_msgs/Vector3.h"
#include "ROSIntegration/Public/geometry_msgs/PoseStamped.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "Tablet_Widget.generated.h"

/**
 * 
 */
UCLASS()
class ROSTEST_API UTablet_Widget : public UUserWidget
{
	GENERATED_BODY()
		//Initializing widget
		virtual bool Initialize();
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* UE_XPosition;
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* UE_YPosition;
public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* UE_ZPosition;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* MAVROS_XPosition;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* MAVROS_YPosition;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* MAVROS_ZPosition;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* Drone_Offboard_Status;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* Drone_Land_Status;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* Battery_Percent;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* FOM_Status;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* FTT_Status;

public:
		UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* Armed;

public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* SelectedDrone;


public:
	UPROPERTY()
		class UTopic* DroneSelectionTopic;
	UPROPERTY()
		class UTopic* DronePositionTopic2;
	UPROPERTY()
		class UTopic* DroneOffsetTopic2;
	UPROPERTY()
		class UTopic* DroneOffboardTopic;
	UPROPERTY()
		class UTopic* DroneLandTopic;
	UPROPERTY()
		class UTopic* DroneFOMTopic;
	UPROPERTY()
		class UTopic* DroneFTTTopic;
	UPROPERTY()
		class UTopic* BatteryPercentageTopic;
	UPROPERTY()
		class UTopic* ArmedTopic;

public:
	// Called every frame
	virtual void NativeTick(const FGeometry& MyGeometry, float DeltaTime=20.0f);
	
};
