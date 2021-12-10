import { useEffect, useState } from "react";
import ROSLIB from "roslib";

type SceneCaptionWithLocation = TrajectoryPoint & Caption;

type TrajectoryPoint = {
  timestamp: number;
  latitude: number;
  longitude: number;
  altitude: number;
};

type Caption = {
  timestamp: number;
  caption: string;
};

export const useRosLib = (websocketUrl = "ws://localhost:9090") => {
  const [Ros, setRos] = useState<ROSLIB.Ros | undefined>(undefined);
  const [currentTime, setCurrentTime] = useState<number>(0);
  // TODO: Allow empty array for captions in CurrentCaption component and remove initial value below
  const [captions, setCaptions] = useState<Caption[]>([
    { timestamp: 0, caption: "remove this later" },
  ]);
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([]);
  const [captionsWithLocation, setCaptionsWithLocation] = useState<
    SceneCaptionWithLocation[]
  >([]);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: websocketUrl,
    });
    ros.on("connection", () => {
      console.log("Connected to websocket server");
      setRos(ros);
    });

    // Listen clock
    const clockListener = new ROSLIB.Topic({
      ros: ros,
      name: "/clock",
      messageType: "rosgraph_msgs/Clock",
    });
    clockListener.subscribe((message: ROSLIB.Message) => {
      try {
        // @ts-expect-error: Message is any but has clock.secs and clock.nsecs attribute.
        const timestamp = message.clock.secs + message.clock.nsecs * 1e-9;
        setCurrentTime(timestamp);
      } catch {
        console.error("Failed to parse clock message.");
        setCurrentTime(0);
      }
    });

    // Listen captions
    const captionsListener = new ROSLIB.Topic({
      ros: ros,
      name: "/scene_viewer/scene_captions",
      messageType: "std_msgs/String",
    });
    captionsListener.subscribe((message: ROSLIB.Message) => {
      try {
        // @ts-expect-error: Message is any but has data attribute.
        const captionsObject = JSON.parse(message.data) as Caption[];
        setCaptions(captionsObject);
      } catch {
        console.error("Failed to parse captions message.");
        setCaptions([]);
      }
    });

    // Listen vehicle_trajectory
    const vehicleTrajectoryListener = new ROSLIB.Topic({
      ros: ros,
      name: "/scene_viewer/vehicle_trajectory",
      messageType: "std_msgs/String",
    });
    vehicleTrajectoryListener.subscribe((message: ROSLIB.Message) => {
      try {
        // @ts-expect-error: Message is any but has data attribute.
        const vehicleTrajectoryObject = JSON.parse(message.data) as [
          number,
          number,
          number,
          number
        ][];
        setTrajectory(
          vehicleTrajectoryObject.map((item) => {
            return {
              timestamp: item[0],
              latitude: item[1],
              longitude: item[2],
              altitude: item[3],
            };
          })
        );
      } catch {
        console.error("Failed to parse vehicle trajectory message.");
        setTrajectory([]);
      }
    });

    // Listen captions
    const captionsWithLocationListener = new ROSLIB.Topic({
      ros: ros,
      name: "/scene_viewer/scene_captions_with_locations",
      messageType: "std_msgs/String",
    });
    captionsWithLocationListener.subscribe((message: ROSLIB.Message) => {
      try {
        setCaptionsWithLocation(
          // @ts-expect-error: Message is any but has data attribute.
          JSON.parse(message.data) as SceneCaptionWithLocation[]
        );
      } catch {
        console.error("Failed to parse captions with location message.");
        setCaptionsWithLocation([]);
      }
    });

    // Specify how to clean up after this effect:
    return function cleanup() {
      setRos(undefined);
      clockListener.unsubscribe();
      captionsListener.unsubscribe();
      vehicleTrajectoryListener.unsubscribe();
      ros.close();
      console.log("Cleanup ros.");
    };
  }, [websocketUrl]);

  const seekToTimestamp = (timestamp: number) => {
    if (!Ros) return;

    // Seek
    const seekService = new ROSLIB.Service({
      ros: Ros,
      name: "/rosbag_player_controller/seek_and_play",
      serviceType: "controllable_rosbag_player/Seek",
    });
    const request = new ROSLIB.ServiceRequest({
      time: timestamp,
    });
    seekService.callService(request, (reponse: ROSLIB.ServiceResponse) => {
      console.log(reponse);
    });
  };

  return {
    Ros,
    currentTime,
    captions,
    trajectory,
    captionsWithLocation,
    seekToTimestamp,
  };
};
