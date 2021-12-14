import { useEffect, useMemo, useRef, useState } from "react";
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

export type UseRosLibArgs = {
  websocketUrl?: string;
  topicNames: TopicNames;
  maxFreq?: number;
};
export const useRosLib = ({
  websocketUrl = "ws://localhost:9090",
  topicNames: initialTopicNames,
  maxFreq = 100,
}: UseRosLibArgs) => {
  const [Ros, setRos] = useState<ROSLIB.Ros | undefined>(undefined);
  const [topicNames, setTopicNames] = useState(initialTopicNames);

  // topic state
  const [currentTime, setCurrentTime] = useState<number>(0);
  const [captions, setCaptions] = useState<Caption[]>([]);
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([]);
  const [captionsWithLocation, setCaptionsWithLocation] = useState<
    SceneCaptionWithLocation[]
  >([]);

  const latestUpdateTimes = useRef<{ [topicName: string]: number }>({});
  const subscriptions: {
    name: TopicNames[number];
    type: string;
    callback: (message: ROSLIB.Message) => void;
    isFrequent?: boolean;
  }[] = useMemo(
    () => [
      {
        name: "/clock",
        type: "rosgraph_msgs/Clock",
        callback: (message) => {
          try {
            // @ts-expect-error: Message is any but has clock.secs and clock.nsecs attribute.
            const timestamp = message.clock.secs + message.clock.nsecs * 1e-9;
            setCurrentTime(timestamp);
          } catch {
            console.error("Failed to parse clock message.");
            setCurrentTime(0);
          }
        },
        isFrequent: true,
      },
      {
        name: "/scene_viewer/scene_captions",
        type: "std_msgs/String",
        callback: (message) => {
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
      },
      {
        name: "/scene_viewer/vehicle_trajectory",
        type: "std_msgs/String",
        callback: (message) => {
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
      },
      {
        name: "/scene_viewer/scene_captions_with_locations",
        type: "std_msgs/String",
        callback: (message) => {
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
      },
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
    subscriptions.forEach(({ name, type, callback, isFrequent }) => {
      if (topicNames.includes(name)) {
        const infrequentCallback = (message: ROSLIB.Message) => {
          const latestTime = latestUpdateTimes.current[name];
          if (latestTime != null && Date.now() - latestTime < maxFreq) {
            return;
          }
          latestUpdateTimes.current[name] = Date.now();
          callback(message);
        };

        const listener = new ROSLIB.Topic({ ros, name, messageType: type });
        listener.subscribe(isFrequent ? infrequentCallback : callback);
        topicListeners.push(listener);
      }
    });

    // Specify how to clean up after this effect:
    return function cleanup() {
      setRos(undefined);
      topicListeners.forEach((topicListener) => topicListener.unsubscribe());
      ros.close();
    };
  }, [websocketUrl, topicNames, subscriptions, maxFreq]);

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
    setTopicNames,
  };
};
