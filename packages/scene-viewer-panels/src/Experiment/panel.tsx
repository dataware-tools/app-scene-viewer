import { useMessageDataItem } from "@foxglove/studio-base/components/MessagePathSyntax/useMessageDataItem";
import {
  MessagePipelineContext,
  useMessagePipeline,
} from "@foxglove/studio-base/components/MessagePipeline";
import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button";
import Stack from "@mui/material/Stack";
import TextField from "@mui/material/TextField";
import { useState, useEffect } from "react";
import styled from "styled-components";
import { Spacer } from "src/components/Spacer";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

type ClickedPoint = {
  header: {
    seq: number;
    stamp: { sec: number; nsec: number };
    frame_id: string;
  };
  point: { x: number; y: number; z: number };
};

type Comment = {
  annotationId: number;
  text: string;
  target: { point: ClickedPoint["point"] };
  frameId: ClickedPoint["header"]["frame_id"];
};

const publishSelector = (ctx: MessagePipelineContext) => ctx.publish;
const setPublishersSelector = (ctx: MessagePipelineContext) =>
  ctx.setPublishers;
const publishersSelector = (ctx: MessagePipelineContext) => ctx.publishers;

const initializeMarker = {
  header: {
    seq: 0,
    stamp: { sec: 0, nsec: 0 },
    frame_id: "map",
  },
  ns: "",
  id: 0,
  type: 2,
  action: 3,
  pose: {
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 0 },
  },
  scale: { x: 5, y: 5, z: 5 },
  color: { r: 1, g: 0.8, b: 0, a: 0.5 },
  lifetime: { sec: 0, nsec: 0 },
  frame_locked: false,
  text: "initializer",
  mesh_resource: "",
  mesh_use_embedded_materials: false,
};
function ExperimentPanel(): JSX.Element {
  const [inputText, setInputText] = useState<string | undefined>(undefined);
  const [editedCommentId, setEditedCommentId] = useState<number | undefined>(
    undefined
  );
  const matchedMessages = useMessageDataItem("/clicked_point");
  const clickedPoint = matchedMessages[matchedMessages.length - 1]
    ?.queriedData[0].value as ClickedPoint | undefined;
  const [comments, setComments] = useState<Comment[]>([
    {
      annotationId: 1,
      frameId: "base_link",
      target: { point: { x: -5, y: 5, z: 0 } },
      text: "example",
    },
    {
      annotationId: 2,
      frameId: "base_link",
      target: { point: { x: 10, y: 7, z: 0 } },
      text: "sample",
    },
  ]);
  const [newAnnotationId, setNewAnnotationId] = useState(3);

  const publishMessage = useMessagePipeline(publishSelector);
  const setPublishers = useMessagePipeline(setPublishersSelector);
  const publishers = useMessagePipeline(publishersSelector);

  const checkIfCommentHighlighted = (comment: Comment) =>
    clickedPoint
      ? Math.sqrt(
          (comment.target.point.x - clickedPoint.point.x) ** 2 +
            (comment.target.point.y - clickedPoint.point.y) ** 2 +
            (comment.target.point.z - clickedPoint.point.z) ** 2
        ) < 3
      : true;

  useEffect(() => {
    setPublishers("comment-click", [
      { topic: "/clicked_point", datatype: "geometry_msgs/PointStamped" },
    ]);
    setPublishers("comment-markers", [
      {
        topic: "/markers/comments",
        datatype: "visualization_msgs/MarkerArray",
      },
    ]);
    setPublishers("comment-labels", [
      {
        topic: "/markers/label-for-comments",
        datatype: "visualization_msgs/MarkerArray",
      },
    ]);
  }, [setPublishers]);

  useEffect(() => {
    if (
      publishers.some((publisher) => publisher.topic === "/markers/comments")
    ) {
      const markers = [
        initializeMarker,
        ...comments.map((comment) => ({
          header: {
            seq: 0,
            stamp: { sec: 0, nsec: 0 },
            frame_id: comment.frameId,
          },
          ns: "",
          id: comment.annotationId,
          type: 2,
          action: 0,
          pose: {
            position: comment.target.point,
            orientation: { x: 0, y: 0, z: 0, w: 0 },
          },
          scale: { x: 5, y: 5, z: 5 },
          color: checkIfCommentHighlighted(comment)
            ? { r: 0.3, g: 0.3, b: 1, a: 1 }
            : { r: 0.4, g: 0.4, b: 0.4, a: 0.5 },
          lifetime: { sec: 0, nsec: 0 },
          frame_locked: false,
          text: comment.text,
          mesh_resource: "",
          mesh_use_embedded_materials: false,
        })),
      ];

      try {
        publishMessage({
          topic: "/markers/comments",
          msg: { markers },
        });
      } catch (error) {
        console.error(error);
      }
    }
    if (
      publishers.some(
        (publisher) => publisher.topic === "/markers/label-for-comments"
      )
    ) {
      const labels = [
        initializeMarker,
        ...comments.map((comment) => ({
          header: {
            seq: 0,
            stamp: { sec: 0, nsec: 0 },
            frame_id: comment.frameId,
          },
          ns: "",
          id: comment.annotationId,
          type: 9,
          action: 0,
          pose: {
            position: comment.target.point,
            orientation: { x: 0, y: 0, z: 0, w: 0 },
          },
          scale: { x: 0, y: 0, z: 5 },
          color: { r: 1, g: 1, b: 1, a: 0.5 },
          lifetime: { sec: 0, nsec: 0 },
          frame_locked: false,
          text: `${comment.annotationId}`,
          mesh_resource: "",
          mesh_use_embedded_materials: false,
        })),
      ];

      try {
        publishMessage({
          topic: "/markers/label-for-comments",
          msg: { markers: labels },
        });
      } catch (error) {
        console.error(error);
      }
    }
  }, [comments.length, publishers, clickedPoint]);

  const Comment = ({ comment }: { comment: Comment }) => (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
      }}
    >
      <Box
        sx={{
          display: "flex",
          flexDirection: "row",
          alignItems: "center",
        }}
      >
        <Box
          sx={{
            borderRadius: "100%",
            border: "solid",
            width: "2rem",
            height: "2rem",
            fontSize: "1rem",
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
          }}
        >
          {comment.annotationId}
        </Box>
        <Spacer horizontal size={2} />
        <TextField
          disabled={editedCommentId !== comment.annotationId}
          value={comment.text}
          variant="outlined"
          onClick={() => setEditedCommentId(comment.annotationId)}
          onChange={(e) =>
            setComments((prev) => {
              const newComments = prev.map((prevComment) =>
                prevComment.annotationId === comment.annotationId
                  ? {
                      ...prevComment,
                      text: e.target.value,
                    }
                  : prevComment
              );

              return newComments;
            })
          }
        />
        <Spacer horizontal size={3} />
        <Button
          variant="contained"
          onClick={() => {
            try {
              publishMessage({
                topic: "/clicked_point",
                msg: {
                  header: {
                    frame_id: comment.frameId,
                  },
                  point: comment.target.point,
                },
              });
            } catch {
              console.error("fail to publish /clicked_point");
            }
          }}
        >
          Select
        </Button>
        <Spacer horizontal size={1} />
        <Button
          variant="contained"
          onClick={() =>
            setComments((prev) => {
              return [
                ...prev.filter(
                  (prevComment) =>
                    prevComment.annotationId !== comment.annotationId
                ),
              ];
            })
          }
          color="error"
        >
          Delete
        </Button>
      </Box>
    </Box>
  );

  return (
    <Container>
      <PanelToolbar floating />
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          fontSize: "1.5rem",
          padding: 3,
        }}
      >
        {clickedPoint ? (
          <>
            <Box
              sx={{
                width: "100%",
                display: "flex",
                flexDirection: "row",
              }}
            >
              <TextField
                variant="standard"
                value={inputText}
                onChange={(e) => setInputText(e.target.value)}
                sx={{ flexGrow: 1 }}
              />
              <Spacer size={3} horizontal />
              <Button
                variant="contained"
                onClick={() => {
                  setComments((prev) => [
                    ...prev,
                    {
                      annotationId: newAnnotationId,
                      text: inputText,
                      target: {
                        point: clickedPoint.point,
                      },
                      // 本当は clickedPoint.header.frame_id が正しいが、デモはこうしないとうまくうごないのでこれにしておく
                      frameId: "base_link",
                    } as typeof prev[number],
                  ]);
                  setNewAnnotationId((prev) => prev + 1);
                  setInputText("");
                }}
              >
                Add
              </Button>
            </Box>
            <Box sx={{ fontSize: "0.8rem" }}>
              clicked point: {JSON.stringify(clickedPoint.point)}
            </Box>
            <Spacer size={3} vertical />
          </>
        ) : null}
        <Stack spacing={2} sx={{ backgroundColor: "Highlight" }}>
          {comments
            .filter((comment) => checkIfCommentHighlighted(comment))
            .map((comment, i) => (
              <Comment key={i} comment={comment} />
            ))}
        </Stack>
        <Spacer vertical size={2} />
        <Stack spacing={2}>
          {comments
            .filter((comment) => !checkIfCommentHighlighted(comment))
            .map((comment, i) => (
              <Comment key={i} comment={comment} />
            ))}
        </Stack>
      </Box>
    </Container>
  );
}

ExperimentPanel.panelType = "CurrentCaptionPanel";
ExperimentPanel.defaultConfig = {};

const Wrapped = Panel(ExperimentPanel);

export { Wrapped as Experiment };
