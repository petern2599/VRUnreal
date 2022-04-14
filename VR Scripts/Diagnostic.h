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
#include "Diagnostic.generated.h"

/**
 * 
 */
UCLASS()
class ROS_VR_API UDiagnostic : public UUserWidget
{
	GENERATED_BODY()
		//Initializing widget
		virtual bool Initialize();
public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* UE4_XPosition;
public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* UE4_YPosition;
public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* UE4_ZPosition;

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
		class UTextBlock* Battery_Percent;



public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* Armed;


public:
	UPROPERTY(meta = (BindWidget), EditAnywhere, BlueprintReadWrite)
		class UTextBlock* State;



public:
	UPROPERTY()
		class UTopic* BatteryPercentageTopic;
	UPROPERTY()
		class UTopic* ArmedTopic;
	UPROPERTY()
		class UTopic* StateTopic;


public:
	// Called every frame
	//virtual void NativeTick(const FGeometry& MyGeometry, float DeltaTime);
	UFUNCTION(BlueprintCallable, Category = "Diagnostic")
		void GetAllDroneStatus(int drone_number);
};
