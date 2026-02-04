{
  "startPoint": {
    "x": 115.99999999999999,
    "y": 131.7241379310345,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-gsgbmzubnru",
      "name": "Path 1",
      "endPoint": {
        "x": 105.44827586206897,
        "y": 33.724137931034484,
        "heading": "linear",
        "startDeg": 37,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#77A757",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7b3t9o-f8shuo",
      "name": "Path 2",
      "endPoint": {
        "x": 38.275862068965516,
        "y": 33.93103448275865,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#D7B95A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml7b4pfp-vjbcyk",
      "name": "Path 3",
      "endPoint": {
        "x": 72,
        "y": 72,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#C5B5AD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-gsgbmzubnru"
    },
    {
      "kind": "wait",
      "id": "ml7b3ony-xrssrk",
      "name": "Wait",
      "durationMs": 5000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "ml7b3t9o-f8shuo"
    },
    {
      "kind": "wait",
      "id": "ml7b4jdx-yj9goz",
      "name": "Wait",
      "durationMs": 5000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "ml7b4pfp-vjbcyk"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-02-04T00:53:56.192Z"
}