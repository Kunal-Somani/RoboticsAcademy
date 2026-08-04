import React, { useState, useEffect } from "react";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

import "./css/GUICanvas.css";

type Readings = {
  ch4: number;
  co: number;
  crack: boolean;
  pose: number[];
};

function WebGUI() {
  const [cameraImage, setCameraImage] = useState<string | undefined>(undefined);
  const [mapImage, setMapImage] = useState<string | undefined>(undefined);
  const [readings, setReadings] = useState<Readings | undefined>(undefined);

  const exerciseContext = useExercise();
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;
    if (!update) {
      return;
    }

    if (update.camera) {
      setCameraImage(`data:image/jpeg;base64,${update.camera}`);
    }

    if (update.map) {
      setMapImage(`data:image/jpeg;base64,${update.map}`);
    }

    if (update.readings) {
      try {
        setReadings(JSON.parse(update.readings));
      } catch {
        // ignore malformed readings frame
      }
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setCameraImage(undefined);
      setMapImage(undefined);
      setReadings(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  const crackLabel = readings && readings.crack ? "CRACK" : "clear";
  const ch4Str = readings ? readings.ch4.toFixed(0) : "--";
  const coStr = readings ? readings.co.toFixed(0) : "--";

  return (
    <WebGUIContainer id="mine-canvas">
      <WebGUIImage
        id="mine_camera_view"
        style={{ left: "0" }}
        src={cameraImage}
      />
      <WebGUIImage
        id="mine_map_view"
        style={{ left: "50%" }}
        src={mapImage}
      />
      <div id="mine_readings_overlay">
        <div>
          <span className="label">CH4:</span> <span>{ch4Str} ppm</span>
        </div>
        <div>
          <span className="label">CO:</span> <span>{coStr} ppm</span>
        </div>
        <div>
          <span className="label">Crack zone:</span>{" "}
          <span className={readings && readings.crack ? "crack-on" : ""}>
            {crackLabel}
          </span>
        </div>
      </div>
    </WebGUIContainer>
  );
}

export default WebGUI;
