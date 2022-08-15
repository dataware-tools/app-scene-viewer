export type ClickedPointType = {
  header: {
    seq: number;
    stamp: { sec: number; nsec: number };
    frame_id: string;
  };
  point: { x: number; y: number; z: number };
};
