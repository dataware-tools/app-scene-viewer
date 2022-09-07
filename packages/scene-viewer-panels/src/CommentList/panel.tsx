import { useMessageDataItem } from "@foxglove/studio-base/components/MessagePathSyntax/useMessageDataItem";
import {
  MessagePipelineContext,
  useMessagePipeline,
} from "@foxglove/studio-base/components/MessagePipeline";
import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import { Typography } from "@mui/material";
import Box from "@mui/material/Box";
import Stack from "@mui/material/Stack";
import { Index } from "flexsearch";
import { useState, useEffect, useRef, useMemo } from "react";
import styled from "styled-components";
import { Comment, CommentType } from "./Comment";
import { CommentInput } from "./CommentInput";
import { SearchBar } from "./SearchBar";
import { ClickedPointType } from "./type";
import { Spacer } from "src/components/Spacer";

const Container = styled.div`
  width: 100%;
  height: 100%;
`;

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

const markerBase = {
  ...initializeMarker,
  action: 0,
};
function CommentListPanel(): JSX.Element {
  const [editedCommentId, setEditedCommentId] = useState<number | undefined>(
    undefined
  );
  const matchedMessages = useMessageDataItem("/clicked_point");
  const clickedPoint = matchedMessages[matchedMessages.length - 1]
    ?.queriedData[0].value as ClickedPointType | undefined;
  const [highlightedTexts, setHighlightedTexts] = useState([""]);
  const [searchText, setSearchText] = useState("");

  // TODO: should be replaced by fetched data
  const [comments, setComments] = useState<CommentType[]>([]);
  const [newAnnotationId, setNewAnnotationId] = useState(1);

  const flexSearchRef = useRef(
    new Index({
      tokenize: "full",
    })
  );
  const searchResult = useMemo(() => {
    return searchText
      ? new Set(flexSearchRef.current.search(searchText))
      : undefined;
  }, [searchText]);

  const publishMessage = useMessagePipeline(publishSelector);
  const setPublishers = useMessagePipeline(setPublishersSelector);
  const publishers = useMessagePipeline(publishersSelector);

  const checkIfCommentHighlighted = (comment: CommentType) =>
    clickedPoint
      ? Math.sqrt(
          (comment.target.point.x - clickedPoint.point.x) ** 2 +
            (comment.target.point.y - clickedPoint.point.y) ** 2 +
            (comment.target.point.z - clickedPoint.point.z) ** 2
        ) < 3
      : false;

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
          ...markerBase,
          header: {
            ...markerBase.header,
            frame_id: comment.frameId,
          },
          id: comment.annotationId,
          pose: {
            ...markerBase.pose,
            position: comment.target.point,
          },
          color: checkIfCommentHighlighted(comment)
            ? { r: 0.3, g: 0.3, b: 1, a: 1 }
            : { r: 1, g: 1, b: 1, a: 0.5 },
          text: comment.text,
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
            ...markerBase.header,
            frame_id: comment.frameId,
          },
          id: comment.annotationId,
          type: 9,
          pose: {
            ...markerBase.pose,
            position: comment.target.point,
          },
          text: `(${comment.annotationId})`,
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

  useEffect(() => {
    comments.forEach(({ text, annotationId }) =>
      flexSearchRef.current.add(annotationId, text)
    );
  }, [comments]);

  // TODO: should be replaced to fetch process
  const updateComment = (
    id: CommentType["annotationId"],
    text: CommentType["text"]
  ) => {
    setComments((prev) => {
      const newComments = prev.map((prevComment) =>
        prevComment.annotationId === id
          ? {
              ...prevComment,
              text,
            }
          : prevComment
      );

      return newComments;
    });
    setEditedCommentId(undefined);
  };

  // TODO: should be replaced to fetch process
  const deleteComment = (id: CommentType["annotationId"]) => {
    setComments((prev) => {
      return [...prev.filter((prevComment) => prevComment.annotationId !== id)];
    });
  };

  const publishClickedPoint = (
    frameId: ClickedPointType["header"]["frame_id"],
    point: ClickedPointType["point"]
  ) => {
    try {
      publishMessage({
        topic: "/clicked_point",
        msg: {
          header: {
            frame_id: frameId,
          },
          point,
        },
      });
    } catch {
      console.error("fail to publish /clicked_point");
    }
  };

  // TODO: should be replaced to fetch process
  const addComment = (comment: string) => {
    if (clickedPoint) {
      setComments((prev) => [
        ...prev,
        {
          annotationId: newAnnotationId,
          text: comment,
          target: {
            point: clickedPoint.point,
          },
          frameId: clickedPoint.header.frame_id,
        } as typeof prev[number],
      ]);
    } else {
      console.error("clicked point is undefined");
    }
    setNewAnnotationId((prev) => prev + 1);
  };

  const searchComment = (searchText: string) => {
    const fixedSearchText = searchText.replace("　", " ");
    setHighlightedTexts(fixedSearchText.split(" "));
    setSearchText(fixedSearchText);
  };

  return (
    <Container>
      <PanelToolbar floating />
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          fontSize: "1.5rem",
          padding: 3,
          height: "100%",
        }}
      >
        <SearchBar onSearch={searchComment} />
        <Stack
          spacing={1}
          sx={{ overflow: "auto", padding: 1, height: "100%" }}
        >
          {comments
            .filter(({ annotationId }) =>
              searchResult ? searchResult.has(annotationId) : true
            )
            .map((comment) => (
              <Comment
                key={comment.annotationId}
                comment={comment}
                highlight={checkIfCommentHighlighted(comment)}
                onStartEdit={() => setEditedCommentId(comment.annotationId)}
                onSaveEdit={(nextCommentText) =>
                  updateComment(comment.annotationId, nextCommentText)
                }
                onCancelEdit={() => setEditedCommentId(undefined)}
                onDelete={() => deleteComment(comment.annotationId)}
                onSelect={() =>
                  publishClickedPoint(comment.frameId, comment.target.point)
                }
                editing={comment.annotationId === editedCommentId}
                highlightedText={highlightedTexts}
              />
            ))}
        </Stack>
        {clickedPoint ? (
          <>
            <Box
              sx={{
                width: "100%",
                display: "flex",
                flexDirection: "row",
              }}
            >
              <CommentInput
                onSave={(comment, clearComment) => {
                  addComment(comment);
                  clearComment();
                }}
                onCancel={(_, clearComment) => {
                  clearComment();
                }}
              />
            </Box>
            <Box sx={{ fontSize: "0.8rem" }}>
              clicked point: {JSON.stringify(clickedPoint.point)}
            </Box>
            <Spacer size={3} vertical />
          </>
        ) : (
          <Typography>
            If you want to add comment, publish point on 3D panel...
          </Typography>
        )}
      </Box>
    </Container>
  );
}

CommentListPanel.panelType = "CommentListPanel";
CommentListPanel.defaultConfig = {};

const Wrapped = Panel(CommentListPanel);

export { Wrapped as CommentList };
