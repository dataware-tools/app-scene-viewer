import Leaflet from "leaflet";
import { useEffect, useState } from "react";
import ROSLIB from "roslib";
import { TimestampCaption } from "../CurrentCaption";

export const useRosLib = (websocketUrl = "ws://localhost:9090") => {
  const [Ros, setRos] = useState<ROSLIB.Ros | undefined>(undefined);
  // TODO: Allow empty array for captions in CurrentCaption component and remove initial value below
  const [captions, setCaptions] = useState<TimestampCaption[]>([
    { timestamp: 0, caption: "remove this later" },
  ]);
  const [trajectory, setTrajectory] = useState<Leaflet.LatLngExpression[]>([]);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: websocketUrl,
    });
    ros.on("connection", () => {
      console.log("Connected to websocket server");
      setRos(ros);
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
        const captionsObject = JSON.parse(message.data) as TimestampCaption[];
        setCaptions(captionsObject);
      } catch {
        console.error("Failed to parse captions message.");
        setCaptions([]);
      }
      captionsListener.unsubscribe();
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
        // TODO (@yusukefs): Add timestamp to trajectory
        setTrajectory(
          vehicleTrajectoryObject.map((item) => [item[1], item[2]])
        );
      } catch {
        console.error("Failed to parse captions message.");
        setTrajectory([]);
      }
      vehicleTrajectoryListener.unsubscribe();
    });

    // Specify how to clean up after this effect:
    return function cleanup() {
      setRos(undefined);
      captionsListener.unsubscribe();
      vehicleTrajectoryListener.unsubscribe();
      ros.close();
      console.log("Cleanup ros.");
    };
  }, [websocketUrl]);

  return { Ros, captions, trajectory };
};
