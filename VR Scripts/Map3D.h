// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Map3D.generated.h"

UCLASS()
class ROS_VR_API AMap3D : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMap3D();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone0_LocationX;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone0_LocationY;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone0_LocationZ;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone1_LocationX;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone1_LocationY;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone1_LocationZ;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone2_LocationX;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone2_LocationY;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DroneLocation")
		float Drone2_LocationZ;

	//public:	
		// Called every frame
		//virtual void Tick(float DeltaTime) override;


public:
	UFUNCTION(BlueprintCallable, Category = "Map3D")
		void GetAllDroneLocations();

};
