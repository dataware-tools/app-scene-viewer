import { useEffect, useMemo, useState } from "react";
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

type TopicNames = (
  | "/clock"
  | "/scene_viewer/scene_captions"
  | "/scene_viewer/vehicle_trajectory"
  | "/scene_viewer/scene_captions_with_locations"
)[];

export type UseRosLibArgs = { websocketUrl?: string; topicNames: TopicNames };
export const useRosLib = ({
  websocketUrl: initialWebsocketUrl = "ws://localhost:9090",
  topicNames: initialTopicNames,
}: UseRosLibArgs) => {
  const [Ros, setRos] = useState<ROSLIB.Ros | undefined>(undefined);

  const [websocketUrl, setWebsocketUrl] = useState(initialWebsocketUrl);
  const [topicNames, setTopicNames] = useState(initialTopicNames);

  // topic state
  const [currentTime, setCurrentTime] = useState<number>(0);
  const [captions, setCaptions] = useState<Caption[]>([]);
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([]);
  const [captionsWithLocation, setCaptionsWithLocation] = useState<
    SceneCaptionWithLocation[]
  >([]);

  const subscriptions: [
    TopicNames[number],
    string,
    (message: ROSLIB.Message) => void
  ][] = useMemo(
    () => [
      [
        "/clock",
        "rosgraph_msgs/Clock",
        (message) => {
          try {
            // @ts-expect-error: Message is any but has clock.secs and clock.nsecs attribute.
            const timestamp = message.clock.secs + message.clock.nsecs * 1e-9;
            setCurrentTime(timestamp);
          } catch {
            console.error("Failed to parse clock message.");
            setCurrentTime(0);
          }
        },
      ],
      [
        "/scene_viewer/scene_captions",
        "std_msgs/String",
        (message) => {
          try {
            // @ts-expect-error: Message is any but has data attribute.
            const captionsObject = JSON.parse(message.data).map((caption) => ({
              ...caption,
              timestamp: parseFloat(caption.timestamp),
            })) as Caption[];
            setCaptions(captionsObject);
          } catch {
            console.error("Failed to parse captions message.");
            setCaptions([]);
          }
        },
      ],
      [
        "/scene_viewer/vehicle_trajectory",
        "std_msgs/String",
        (message) => {
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
        },
      ],
      [
        "/scene_viewer/scene_captions_with_locations",
        "std_msgs/String",
        (message) => {
          try {
            setCaptionsWithLocation(
              // @ts-expect-error: Message is any but has data attribute.
              JSON.parse(message.data) as SceneCaptionWithLocation[]
            );
          } catch {
            console.error("Failed to parse captions with location message.");
            setCaptionsWithLocation([]);
          }
        },
      ],
    ],
    []
  );

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: websocketUrl,
    });
    ros.on("connection", () => {
      setRos(ros);
    });

    const topicListeners: ROSLIB.Topic<ROSLIB.Message>[] = [];
    subscriptions.forEach(([name, messageType, callback]) => {
      if (topicNames.includes(name)) {
        const listener = new ROSLIB.Topic({ ros, name, messageType });
        listener.subscribe(callback);
        topicListeners.push(listener);
      }
    });

    // Specify how to clean up after this effect:
    return function cleanup() {
      setRos(undefined);
      topicListeners.forEach((topicListener) => topicListener.unsubscribe());
      ros.close();
    };
  }, [websocketUrl, topicNames, subscriptions]);

  const seekToTimestamp = (timestamp: number) => {
    if (!Ros) return;

    // Seek
    const seekService = new ROSLIB.Service({
      ros: Ros,
      name: "/rosbag_player_controller/seek",
      serviceType: "controllable_rosbag_player/Seek",
    });
    const request = new ROSLIB.ServiceRequest({
      time: timestamp,
    });
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    seekService.callService(request, () => {});
  };

  return {
    Ros,
    currentTime,
    captions,
    trajectory,
    captionsWithLocation,
    seekToTimestamp,
    setWebsocketUrl,
    setTopicNames,
  };
};
