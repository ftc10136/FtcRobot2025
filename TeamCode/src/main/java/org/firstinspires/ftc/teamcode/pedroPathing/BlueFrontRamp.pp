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
        "x": 59.7,
        "y": 77.4,
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
        "y": 59.5,
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
        "x": 17.2,
        "y": 59.5,
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
        "x": 59.7,
        "y": 77.4,
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
    },
    {
      "id": "mo4nywrg-gji785",
      "name": "Path 6",
      "endPoint": {
        "x": 13.5,
        "y": 60.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 148
      },
      "controlPoints": [],
      "color": "#66A5CC",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo4o0zec-iac91k",
      "name": "Path 7",
      "endPoint": {
        "x": 59.7,
        "y": 77.4,
        "heading": "constant",
        "reverse": false,
        "degrees": 148
      },
      "controlPoints": [],
      "color": "#A57D7B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo4p0erb-xphgpq",
      "name": "Path 8",
      "endPoint": {
        "x": 40.75,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 148,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#586CD6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo4p1aks-d441co",
      "name": "Path 9",
      "endPoint": {
        "x": 17.2,
        "y": 84,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#ACBACD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mo4p33fd-3y1jkv",
      "name": "Path 10",
      "endPoint": {
        "x": 58.7,
        "y": 100,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 135
      },
      "controlPoints": [],
      "color": "#D999D9",
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
    },
    {
      "kind": "path",
      "lineId": "mo4nywrg-gji785"
    },
    {
      "kind": "path",
      "lineId": "mo4o0zec-iac91k"
    },
    {
      "kind": "path",
      "lineId": "mo4p0erb-xphgpq"
    },
    {
      "kind": "path",
      "lineId": "mo4p1aks-d441co"
    },
    {
      "kind": "path",
      "lineId": "mo4p33fd-3y1jkv"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-04-18T19:32:06.276Z"
}