{
  "startPoint": {
    "x": 18.137931034482754,
    "y": 119.51724137931035,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-kq1u3ld67dp",
      "name": "Start",
      "endPoint": {
        "x": 59.724137931034484,
        "y": 77.37931034482757,
        "heading": "linear",
        "startDeg": 143.5,
        "endDeg": 218,
        "degrees": 143.5
      },
      "controlPoints": [],
      "color": "#68DB75",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo3kmdn4-jkgm8p",
      "name": "Path 2",
      "endPoint": {
        "x": 40.758620689655174,
        "y": 60.37931034482756,
        "heading": "linear",
        "reverse": false,
        "degrees": 180,
        "startDeg": 218,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#757B75",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo3kmzp8-lfy5sh",
      "name": "Path 3",
      "endPoint": {
        "x": 17.17241379310346,
        "y": 59.48275862068965,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#B86565",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo3rghc8-d7avty",
      "endPoint": {
        "x": 15.8762068965507,
        "y": 66.86206896551724,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 26.327586206896555,
          "y": 66.37931034482757
        }
      ],
      "color": "#658596",
      "name": "Path 4",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo3l0ba2-my7gl1",
      "name": "Path 5",
      "endPoint": {
        "x": 59.724137931034484,
        "y": 77.55172413793105,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 44.465517241379295,
          "y": 70.48275862068967
        }
      ],
      "color": "#5975A9",
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
      "lineId": "line-kq1u3ld67dp"
    },
    {
      "kind": "path",
      "lineId": "mo3kmdn4-jkgm8p"
    },
    {
      "kind": "path",
      "lineId": "mo3kmzp8-lfy5sh"
    },
    {
      "kind": "path",
      "lineId": "mo3rghc8-d7avty"
    },
    {
      "kind": "path",
      "lineId": "mo3l0ba2-my7gl1"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-04-18T03:51:17.270Z"
}