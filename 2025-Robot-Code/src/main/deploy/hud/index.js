import { NT4_Client } from './NT4.js';

const reefTopicName = 'ClosestFace';
const selectedReefColor = "transparent transparent #B00020 transparent"
const unselectedReefColor = "transparent transparent transparent transparent"

console.log("Connecting to NT....")
const ntClient = new NT4_Client(
    window.location.hostname,
    "ReefView",
    (topic) => {

    },
    (topic) => {

    },
    (topic, timestamp, value) => {
        if (topic.name === reefTopicName) {
            currentFace = value;
        } else {
            return
        }
        document.body.style.backgroundColor = "white";
    },
    (topic) => {

    },
    (topic) => {
        document.body.style.backgroundColor = "#36454F"
        console.log("Connection to NetworkTables has been lost!");
    }
);

window.addEventListener("load", () => {
    ntClient.subscribe(
        [
            reefTopicName,
        ],
        false,
        false, 
        0.02
    );
    ntClient.connect();
});

let currentFace = "A";
// A = top, 
// B = top-right, 
// C = bottom-right,
// D = bottom
// E = bottom-left,
// F = top-left

function updateUI() {
    Array.from(document.getElementsByClassName("triangle")).forEach(
    (triangle, index) => {
        switch(currentFace) {
            case "A":
                if (index === 0) {
                    triangle.style.borderColor = selectedReefColor;
                } else {
                    triangle.style.borderColor = unselectedReefColor;
                }
                break;
            case "B":
                if (index === 1) {
                    triangle.style.borderColor = selectedReefColor
                } else {
                    triangle.style.borderColor = unselectedReefColor;
                }
                break;
            case "C":
                if (index === 2) {
                    triangle.style.borderColor = selectedReefColor;
                } else {
                    triangle.style.borderColor = unselectedReefColor;
                }
                break;
            case "D":
                if (index === 3) {
                    triangle.style.borderColor = selectedReefColor;
                } else {
                    triangle.style.borderColor = unselectedReefColor;
                }
                break;
            case "E":
                if (index === 4) {
                    triangle.style.borderColor = selectedReefColor;
                } else {
                    triangle.style.borderColor = unselectedReefColor;
                }
                break;
            case "F":
                if (index === 5) {
                    triangle.style.borderColor = selectedReefColor;
                } else {
                    triangle.style.borderColor = unselectedReefColor;
                }
                break;
            default:
                triangle.style.borderColor = unselectedReefColor;
        }
    });
}