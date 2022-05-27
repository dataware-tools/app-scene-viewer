import { css } from "@emotion/css";
import { Story } from "@storybook/react";
import { useRosLib } from "../hooks/roslibHooks";
import { SceneSelector, SceneSelectorProps } from "./content";

export default {
  component: SceneSelector,
  title: "SceneSelector",
};

const Template: Story<SceneSelectorProps> = (args) => (
  <div
    className={css`
      height: 500px;
      width: 800px;
    `}
  >
    <SceneSelector {...args} />
  </div>
);

export const Default = Template.bind({});
Default.args = {
  captions: [
    {
      timestamp: 11111,
      caption:
        "しかし、こうした件は全部が重要ではない。もっと重要なのは、 昔チャップリンは不意にこう言いました、「アイデアは、それを一心に求めてさえいれば必ず生まれる。」",
      location: { altitude: 1, latitude: 1, longitude: 1 },
    },
    {
      timestamp: 22222,
      caption:
        " でしたら、 わっしょいを発生するには、一体どうやって実現できるのか。",
    },
    {
      timestamp: 33333,
      caption:
        "我々はそれが現れたと言う事実を考えなくてはいけないです。 考え直してみれば、私にとって",
    },
    {
      timestamp: 44444,
      caption:
        "一方、わっしょいを発生させない場合、何を通じてそれをできるのでしょうか",
      location: { altitude: 4, latitude: 4, longitude: 4 },
    },
    {
      timestamp: 55555,
      caption:
        "「いかんものは、いくら考えてもよくならん。」短いながら、この言葉は私に様々な考えを持たせます。",
    },
    {
      timestamp: 66666,
      caption:
        "「およそ人を扱う場合には、相手を論理の動物だと思ってはならない。相手は感情の動物であり、しかも偏見に満ち、自尊心と虚栄心によって行動するということを、よく心得ておかねばならない。」こうした中、私の疑問が解けました。一般論を述べると、問題のコツをマスターすれば、残りは全て刃を迎えて解くと思われます。",
    },
    {
      timestamp: 77777,
      caption: "wasshoi wahhoi uhhoi uhoho",
      location: { altitude: 7, latitude: 7, longitude: 7 },
    },
    {
      timestamp: 88888,
      caption: "foo bar hoge huga uhoho",
      location: { altitude: 8, latitude: 8, longitude: 8 },
    },
    {
      timestamp: 99999,
      caption: "yes no soso foo",
    },
  ],
  setPinLocations: (pinLocations) => {
    console.log("setPinLocations!!");
    console.log(pinLocations);
  },
  onSelectScene: (timestamp) => {
    window.confirm(`Seek to ${timestamp}`);
  },
};

export const WithRos = () => {
  const { captionsWithLocation, seekToTimestamp } = useRosLib({
    topicNames: ["/scene_viewer/scene_captions_with_locations"],
  });
  console.log(captionsWithLocation);
  return (
    <div
      className={css`
        height: 300px;
        width: 800px;
      `}
    >
      <SceneSelector
        captions={captionsWithLocation.map((item) => {
          return {
            timestamp: item.timestamp,
            caption: item.caption,
            location: {
              altitude: item.altitude,
              latitude: item.latitude,
              longitude: item.longitude,
            },
          };
        })}
        setPinLocations={(pinLocations) => {
          console.log("setPinLocations!!");
          console.log(pinLocations);
        }}
        onSelectScene={(timestamp) => {
          seekToTimestamp(timestamp);
        }}
      />
    </div>
  );
};
