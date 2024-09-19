/*
  Enhanced GNSS Example for RP2040 Pico W with Real-Time Web Server, Position Offsets, CSV Upload, and Survey Functionality

  Features:
  - Connects to Wi-Fi (FarmSpaceIOT with password legolego)
  - Connects to RTCM server rtk.farmspace.tech:1025
  - Relays RTCM data to GNSS module over I2C
  - Retrieves and logs high-precision ECEF coordinates at 10 Hz
  - Retrieves and logs Relative Position (RELPOSNED) data
  - Logs fix type and position accuracy
  - Prints satellite summary data without individual satellite logs
  - Serves a dynamic, styled webpage displaying the latest GPS data with real-time updates
  - Displays current offset from target position in RELPOSNED coordinates
  - Includes a list of position offsets that can be navigated and applied
  - Provides a button to set target position to current position
  - Allows adding new position offsets via the web interface
  - Supports uploading a CSV file with a list of position offsets
  - Adds survey functionality to collect and average position data over a specified duration

  Author: [Your Name]
  Date: [Current Date]
  License: MIT
*/

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <WiFi.h>          // WiFi library for RP2040 Pico W
#include <WebServer.h>     // Simple WebServer

// Initialize the GNSS object
SFE_UBLOX_GNSS myGNSS;

// ======= Wi-Fi and RTCM Server Configuration =======

// Wi-Fi credentials
const char* ssid = "FarmSpaceIOT";
const char* wifiPassword = "legolego";

// RTCM server details
const char* rtkServer = "rtk.farmspace.tech";
const uint16_t rtkPort = 1025;

// Create a WiFiClient object for TCP connection
WiFiClient client;

// Define GNSS I2C address (Replace with your GNSS module's actual I2C address)
#define GNSS_I2C_ADDRESS 0x42 // Example address; check your module's datasheet

// =====================================================

// Initialize WebServer on port 80
WebServer server(80);

// ======= GPS Data Variables =======

// ECEF Coordinates
double ECEFX = 0.0;
double ECEFY = 0.0;
double ECEFZ = 0.0;
String fixTypeStr = "UNKNOWN";
float accuracy = 0.0;

// RELPOSNED Data
double relPosN = 0.0;
double relPosE = 0.0;
double relPosD = 0.0;
float relPosHPN = 0.0;
float relPosHPE = 0.0;
float relPosHPD = 0.0;
float accN = 0.0;
float accE = 0.0;
float accD = 0.0;
bool gnssFixOk = false;
bool diffSolution = false;
bool relPosValid = false;
String carrierSolutionType = "None";
bool isMoving = false;

// Satellite Summary
int totalSats = 0;
int satsUsed = 0;
int satsWithCorrections = 0;
int sbasCorrUsed = 0, rtcmCorrUsed = 0, slasCorrUsed = 0, spartnCorrUsed = 0, prCorrUsed = 0, crCorrUsed = 0, doCorrUsed = 0;

// ======= Target Position Variables =======

// Base Target Position (Relative Coordinates)
double targetRelPosN = 0.0;
double targetRelPosE = 0.0;
double targetRelPosD = 0.0;

// Offset from Target Position
double offsetN = 0.0;
double offsetE = 0.0;
double offsetD = 0.0;

// ======= Position Offsets List =======

struct PositionOffset {
  double N;
  double E;
  double D;
};

#define MAX_OFFSETS 20
PositionOffset offsets[MAX_OFFSETS];
int numOffsets = 0;
int currentOffsetIndex = -1; // -1 means no offset applied

// ======= Survey Variables =======

bool isSurveying = false;
unsigned long surveyStartTime = 0;
unsigned long surveyDuration = 10000; // 10 seconds in milliseconds

// Accumulators for averaging
double sumSurveyN = 0.0;
double sumSurveyE = 0.0;
double sumSurveyD = 0.0;
int surveyCount = 0;

// Averaged Survey Position
double avgSurveyN = 0.0;
double avgSurveyE = 0.0;
double avgSurveyD = 0.0;
double surveyOffsetN = 0.0;
double surveyOffsetE = 0.0;
double surveyOffsetD = 0.0;

// ======= Timing Variables =======
unsigned long lastECEFTimestamp = 0;
const unsigned long ECEF_INTERVAL = 100; // 100ms for 10 Hz

// ======= Reconnection Variables =======
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000; // Check Wi-Fi every 5 seconds

// ======= Function Prototypes =======
void handleRoot();
void handleData();
void handleSetTarget();
void handleNextOffset();
void handlePrevOffset();
void handleSelectOffset();
void handleAddOffset();
void handleUploadCSV();
void handleStartSurvey();
void handleStopSurvey();
void handleNotFound();
void newNAVSATSummary(UBX_NAV_SAT_data_t *ubxDataStruct);
String getFixTypeText(uint8_t fixType);
void connectToWiFi();
void reconnectRTCM();

// ======= Web Server Handlers =======

// Handler for root path "/"
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Pico W GNSS Data Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f4f7f9;
      color: #333;
      margin: 0;
      padding: 20px;
    }
    h1, h2 {
      color: #4CAF50;
    }
    .container {
      max-width: 1000px;
      margin: auto;
      background: #fff;
      padding: 30px;
      border-radius: 8px;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
    }
    .section {
      margin-bottom: 30px;
    }
    .section ul {
      list-style-type: none;
      padding: 0;
    }
    .section li {
      padding: 8px 0;
      border-bottom: 1px solid #e0e0e0;
    }
    .section li:last-child {
      border-bottom: none;
    }
    .label {
      font-weight: bold;
    }
    .status-ok {
      color: green;
    }
    .status-fail {
      color: red;
    }
    #offsetSection {
      background-color: #e8f5e9;
      padding: 15px;
      border-radius: 5px;
      margin-bottom: 30px;
      text-align: center;
    }
    #setTargetBtn {
      background-color: #4CAF50;
      color: white;
      padding: 10px 20px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      margin-top: 10px;
    }
    #setTargetBtn:hover {
      background-color: #45a049;
    }
    #offsetsList {
      max-height: 200px;
      overflow-y: auto;
      border: 1px solid #ccc;
      border-radius: 5px;
      padding: 10px;
      margin-top: 20px;
    }
    .offset-item {
      padding: 8px;
      border-bottom: 1px solid #ddd;
      cursor: pointer;
    }
    .offset-item:last-child {
      border-bottom: none;
    }
    .offset-item:hover {
      background-color: #f1f1f1;
    }
    .offset-item.selected {
      background-color: #d4edda;
    }
    #navigationButtons {
      margin-top: 10px;
    }
    #navigationButtons button {
      background-color: #008CBA;
      color: white;
      padding: 8px 16px;
      margin: 5px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      font-size: 14px;
    }
    #navigationButtons button:hover {
      background-color: #007B9E;
    }
    #addOffsetForm {
      margin-top: 20px;
    }
    #addOffsetForm input {
      padding: 8px;
      margin: 5px;
      width: 80px;
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    #addOffsetForm button {
      background-color: #4CAF50;
      color: white;
      padding: 8px 16px;
      margin: 5px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      font-size: 14px;
    }
    #addOffsetForm button:hover {
      background-color: #45a049;
    }
    #uploadCSVForm {
      margin-top: 20px;
    }
    #uploadCSVForm input[type="file"] {
      padding: 8px;
      margin: 5px 0;
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    #uploadCSVForm button {
      background-color: #FF9800;
      color: white;
      padding: 8px 16px;
      margin: 5px 0;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      font-size: 14px;
    }
    #uploadCSVForm button:hover {
      background-color: #FB8C00;
    }
    #surveySection {
      background-color: #e3f2fd;
      padding: 15px;
      border-radius: 5px;
      margin-top: 30px;
    }
    #startSurveyBtn {
      background-color: #FF5722;
      color: white;
      padding: 10px 20px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      margin-top: 10px;
    }
    #startSurveyBtn:hover {
      background-color: #E64A19;
    }
    #startSurveyBtn.active {
      background-color: #f44336;
    }
    #startSurveyBtn.active:hover {
      background-color: #da190b;
    }
    #surveyStatus {
      margin-top: 10px;
      font-weight: bold;
    }
    #surveyResults {
      margin-top: 10px;
      padding: 10px;
      background-color: #fff3e0;
      border-radius: 5px;
      border: 1px solid #ffccbc;
    }
    footer {
      text-align: center;
      margin-top: 40px;
      color: #777;
    }
    @media (max-width: 600px) {
      body {
        padding: 10px;
      }
      #setTargetBtn, #navigationButtons button, #addOffsetForm button, #uploadCSVForm button, #startSurveyBtn {
        width: 100%;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>RP2040 Pico W GNSS Data Dashboard</h1>
    
    <div id="offsetSection" class="section">
      <h2>Current Offset from Target Position</h2>
      <ul>
        <li><span class="label">Offset N (m):</span> <span id="offsetN">Loading...</span></li>
        <li><span class="label">Offset E (m):</span> <span id="offsetE">Loading...</span></li>
        <li><span class="label">Offset D (m):</span> <span id="offsetD">Loading...</span></li>
      </ul>
      <button id="setTargetBtn">Set Current Position as Target</button>
    </div>
    
    <div class="section">
      <h2>ECEF Coordinates</h2>
      <ul>
        <li><span class="label">X (m):</span> <span id="ECEFX">Loading...</span></li>
        <li><span class="label">Y (m):</span> <span id="ECEFY">Loading...</span></li>
        <li><span class="label">Z (m):</span> <span id="ECEFZ">Loading...</span></li>
      </ul>
    </div>
    
    <div class="section">
      <h2>Fix Information</h2>
      <ul>
        <li><span class="label">Fix Type:</span> <span id="fixTypeStr">Loading...</span></li>
        <li><span class="label">Accuracy (m):</span> <span id="accuracy">Loading...</span></li>
      </ul>
    </div>
    
    <div class="section">
      <h2>Relative Position (RELPOSNED)</h2>
      <ul>
        <li><span class="label">relPosN (m):</span> <span id="relPosN">Loading...</span></li>
        <li><span class="label">relPosE (m):</span> <span id="relPosE">Loading...</span></li>
        <li><span class="label">relPosD (m):</span> <span id="relPosD">Loading...</span></li>
        <li><span class="label">relPosHPN (mm):</span> <span id="relPosHPN">Loading...</span></li>
        <li><span class="label">relPosHPE (mm):</span> <span id="relPosHPE">Loading...</span></li>
        <li><span class="label">relPosHPD (mm):</span> <span id="relPosHPD">Loading...</span></li>
        <li><span class="label">accN (m):</span> <span id="accN">Loading...</span></li>
        <li><span class="label">accE (m):</span> <span id="accE">Loading...</span></li>
        <li><span class="label">accD (m):</span> <span id="accD">Loading...</span></li>
        <li><span class="label">gnssFixOk:</span> <span id="gnssFixOk">Loading...</span></li>
        <li><span class="label">diffSolution:</span> <span id="diffSolution">Loading...</span></li>
        <li><span class="label">relPosValid:</span> <span id="relPosValid">Loading...</span></li>
        <li><span class="label">Carrier Solution Type:</span> <span id="carrierSolutionType">Loading...</span></li>
        <li><span class="label">isMoving:</span> <span id="isMoving">Loading...</span></li>
      </ul>
    </div>
    
    <div class="section">
      <h2>Satellite Summary</h2>
      <ul>
        <li><span class="label">Total Satellites:</span> <span id="totalSats">Loading...</span></li>
        <li><span class="label">Satellites Used for Navigation:</span> <span id="satsUsed">Loading...</span></li>
        <li><span class="label">Satellites with Corrections:</span> <span id="satsWithCorrections">Loading...</span></li>
        <li><span class="label">SBAS Corrections Used:</span> <span id="sbasCorrUsed">Loading...</span></li>
        <li><span class="label">RTCM Corrections Used:</span> <span id="rtcmCorrUsed">Loading...</span></li>
        <li><span class="label">SLAS Corrections Used:</span> <span id="slasCorrUsed">Loading...</span></li>
        <li><span class="label">SPARTN Corrections Used:</span> <span id="spartnCorrUsed">Loading...</span></li>
        <li><span class="label">Pseudorange Corrections Used:</span> <span id="prCorrUsed">Loading...</span></li>
        <li><span class="label">Carrier Range Corrections Used:</span> <span id="crCorrUsed">Loading...</span></li>
        <li><span class="label">Doppler Corrections Used:</span> <span id="doCorrUsed">Loading...</span></li>
      </ul>
    </div>
    
    <div class="section">
      <h2>Position Offsets</h2>
      <div id="offsetsList">
        <!-- Offset items will be populated here -->
      </div>
      <div id="navigationButtons">
        <button onclick="prevOffset()">Previous</button>
        <button onclick="nextOffset()">Next</button>
      </div>
      <form id="addOffsetForm">
        <h3>Add New Offset</h3>
        <input type="number" step="0.01" id="newOffsetN" placeholder="N (m)" required>
        <input type="number" step="0.01" id="newOffsetE" placeholder="E (m)" required>
        <input type="number" step="0.01" id="newOffsetD" placeholder="D (m)" required>
        <button type="submit">Add Offset</button>
      </form>
      <form id="uploadCSVForm">
        <h3>Upload Offsets via CSV</h3>
        <input type="file" id="csvFileInput" accept=".csv">
        <button type="button" onclick="uploadCSV()">Upload CSV</button>
      </form>
    </div>
    
    <div class="section" id="surveySection">
      <h2>Survey Functionality</h2>
      <button id="startSurveyBtn" onclick="toggleSurvey()">Start Survey</button>
      <div id="surveyStatus">Status: Idle</div>
      <div id="surveyResults">
        <h3>Survey Results:</h3>
        <ul>
          <li><span class="label">Averaged relPosN (m):</span> <span id="avgSurveyN">N/A</span></li>
          <li><span class="label">Averaged relPosE (m):</span> <span id="avgSurveyE">N/A</span></li>
          <li><span class="label">Averaged relPosD (m):</span> <span id="avgSurveyD">N/A</span></li>
          <li><span class="label">Offset from Target N (m):</span> <span id="surveyOffsetN">N/A</span></li>
          <li><span class="label">Offset from Target E (m):</span> <span id="surveyOffsetE">N/A</span></li>
          <li><span class="label">Offset from Target D (m):</span> <span id="surveyOffsetD">N/A</span></li>
        </ul>
      </div>
    </div>
    
    <footer>
      <p>RP2040 Pico W GNSS Data Dashboard</p>
    </footer>
    
  </div>
  
  <script>
    let surveyInProgress = false;

    async function fetchData() {
      try {
        const response = await fetch('/data');
        const data = await response.json();
        console.log(data);
        
        // Offset from Target Position
        document.getElementById('offsetN').textContent = data.offsetN.toFixed(4);
        document.getElementById('offsetE').textContent = data.offsetE.toFixed(4);
        document.getElementById('offsetD').textContent = data.offsetD.toFixed(4);
        
        // ECEF Coordinates
        document.getElementById('ECEFX').textContent = data.ECEFX.toFixed(4);
        document.getElementById('ECEFY').textContent = data.ECEFY.toFixed(4);
        document.getElementById('ECEFZ').textContent = data.ECEFZ.toFixed(4);
        
        // Fix Information
        document.getElementById('fixTypeStr').textContent = data.fixType;
        document.getElementById('accuracy').textContent = data.accuracy.toFixed(3);
        
        // RELPOSNED Data
        document.getElementById('relPosN').textContent = data.relPosN.toFixed(4);
        document.getElementById('relPosE').textContent = data.relPosE.toFixed(4);
        document.getElementById('relPosD').textContent = data.relPosD.toFixed(4);
        document.getElementById('relPosHPN').textContent = data.relPosHPN.toFixed(1);
        document.getElementById('relPosHPE').textContent = data.relPosHPE.toFixed(1);
        document.getElementById('relPosHPD').textContent = data.relPosHPD.toFixed(1);
        document.getElementById('accN').textContent = data.accN.toFixed(4);
        document.getElementById('accE').textContent = data.accE.toFixed(4);
        document.getElementById('accD').textContent = data.accD.toFixed(4);
        
        // Status Indicators
        document.getElementById('gnssFixOk').textContent = data.gnssFixOk ? '✔️' : '❌';
        document.getElementById('diffSolution').textContent = data.diffSolution ? '✔️' : '❌';
        document.getElementById('relPosValid').textContent = data.relPosValid ? '✔️' : '❌';
        document.getElementById('carrierSolutionType').textContent = data.carrierSolutionType;
        document.getElementById('isMoving').textContent = data.isMoving ? '✔️' : '❌';
        
        // Satellite Summary
        document.getElementById('totalSats').textContent = data.totalSats;
        document.getElementById('satsUsed').textContent = data.satsUsed;
        document.getElementById('satsWithCorrections').textContent = data.satsWithCorrections;
        document.getElementById('sbasCorrUsed').textContent = data.sbasCorrUsed;
        document.getElementById('rtcmCorrUsed').textContent = data.rtcmCorrUsed;
        document.getElementById('slasCorrUsed').textContent = data.slasCorrUsed;
        document.getElementById('spartnCorrUsed').textContent = data.spartnCorrUsed;
        document.getElementById('prCorrUsed').textContent = data.prCorrUsed;
        document.getElementById('crCorrUsed').textContent = data.crCorrUsed;
        document.getElementById('doCorrUsed').textContent = data.doCorrUsed;
        
        // Populate Offsets List
        populateOffsetsList(data.offsets, data.currentOffsetIndex);

        // Update Survey Results if available
        if(data.avgSurveyN !== null){
          document.getElementById('avgSurveyN').textContent = data.avgSurveyN.toFixed(4);
          document.getElementById('avgSurveyE').textContent = data.avgSurveyE.toFixed(4);
          document.getElementById('avgSurveyD').textContent = data.avgSurveyD.toFixed(4);
          document.getElementById('surveyOffsetN').textContent = data.surveyOffsetN.toFixed(4);
          document.getElementById('surveyOffsetE').textContent = data.surveyOffsetE.toFixed(4);
          document.getElementById('surveyOffsetD').textContent = data.surveyOffsetD.toFixed(4);
        }

        // Update Survey Button and Status
        surveyInProgress = data.isSurveying;
        const surveyBtn = document.getElementById('startSurveyBtn');
        const surveyStatus = document.getElementById('surveyStatus');

        if(surveyInProgress){
          surveyBtn.textContent = 'Stop Survey';
          surveyBtn.classList.add('active');
          surveyStatus.textContent = 'Status: Surveying...';
        }
        else{
          surveyBtn.textContent = 'Start Survey';
          surveyBtn.classList.remove('active');
          if(data.surveyCompleted){
            surveyStatus.textContent = 'Status: Survey Completed.';
          }
          else{
            surveyStatus.textContent = 'Status: Idle';
          }
        }
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    }
    
    // Populate the Offsets List in the Webpage
    function populateOffsetsList(offsets, currentIndex) {
      const offsetsList = document.getElementById('offsetsList');
      offsetsList.innerHTML = '';
      
      offsets.forEach((offset, index) => {
        const div = document.createElement('div');
        div.className = 'offset-item';
        if(index === currentIndex){
          div.classList.add('selected');
        }
        div.textContent = `Offset ${index + 1}: N=${offset.N.toFixed(2)}m, E=${offset.E.toFixed(2)}m, D=${offset.D.toFixed(2)}m`;
        div.addEventListener('click', () => {
          selectOffset(index);
        });
        offsetsList.appendChild(div);
      });
    }
    
    // Select an Offset by Index
    async function selectOffset(index) {
      try {
        const response = await fetch(`/select_offset?index=${index}`);
        if (response.ok) {
          console.log(`Selected offset ${index + 1}`);
          fetchData(); // Refresh data
        } else {
          console.error('Failed to select offset.');
        }
      } catch (error) {
        console.error('Error selecting offset:', error);
      }
    }
    
    // Navigate to Next Offset
    async function nextOffset() {
      try {
        const response = await fetch('/next_offset');
        if (response.ok) {
          console.log('Moved to next offset');
          fetchData(); // Refresh data
        } else {
          console.error('Failed to move to next offset.');
        }
      } catch (error) {
        console.error('Error moving to next offset:', error);
      }
    }
    
    // Navigate to Previous Offset
    async function prevOffset() {
      try {
        const response = await fetch('/prev_offset');
        if (response.ok) {
          console.log('Moved to previous offset');
          fetchData(); // Refresh data
        } else {
          console.error('Failed to move to previous offset.');
        }
      } catch (error) {
        console.error('Error moving to previous offset:', error);
      }
    }
    
    // Handle Add Offset Form Submission
    document.getElementById('addOffsetForm').addEventListener('submit', async (e) => {
      e.preventDefault();
      const N = parseFloat(document.getElementById('newOffsetN').value);
      const E = parseFloat(document.getElementById('newOffsetE').value);
      const D = parseFloat(document.getElementById('newOffsetD').value);
      
      try {
        const response = await fetch(`/add_offset?N=${N}&E=${E}&D=${D}`);
        if (response.ok) {
          alert('Offset added successfully.');
          document.getElementById('newOffsetN').value = '';
          document.getElementById('newOffsetE').value = '';
          document.getElementById('newOffsetD').value = '';
          fetchData(); // Refresh data
        } else {
          const errorText = await response.text();
          alert(`Failed to add offset. ${errorText}`);
        }
      } catch (error) {
        console.error('Error adding offset:', error);
        alert('Error adding offset.');
      }
    });
    
    // Handle CSV Upload
    async function uploadCSV() {
      const fileInput = document.getElementById('csvFileInput');
      if (fileInput.files.length === 0) {
        alert('Please select a CSV file to upload.');
        return;
      }
      const file = fileInput.files[0];
      const reader = new FileReader();
      
      reader.onload = async function(e) {
        const csvContent = e.target.result;
        try {
          const response = await fetch('/upload_csv', {
            method: 'POST',
            headers: {
              'Content-Type': 'text/plain',
            },
            body: csvContent
          });
          if (response.ok) {
            const responseText = await response.text();
            alert(responseText);
            fileInput.value = '';
            fetchData(); // Refresh data
          } else {
            const errorText = await response.text();
            alert(`Failed to upload CSV. ${errorText}`);
          }
        } catch (error) {
          console.error('Error uploading CSV:', error);
          alert('Error uploading CSV.');
        }
      };
      
      reader.readAsText(file);
    }
    
    // Toggle Survey (Start/Stop)
    async function toggleSurvey() {
      if(!surveyInProgress){
        // Start Survey
        try {
          const response = await fetch('/start_survey');
          if (response.ok) {
            console.log('Survey started.');
            fetchData(); // Refresh data to update button state
          } else {
            alert('Failed to start survey.');
          }
        } catch (error) {
          console.error('Error starting survey:', error);
          alert('Error starting survey.');
        }
      }
      else{
        // Stop Survey
        try {
          const response = await fetch('/stop_survey');
          if (response.ok) {
            console.log('Survey stopped.');
            fetchData(); // Refresh data to update button state
          } else {
            alert('Failed to stop survey.');
          }
        } catch (error) {
          console.error('Error stopping survey:', error);
          alert('Error stopping survey.');
        }
      }
    }

    // Handle setting target position via button
    async function setTargetPosition() {
      try {
        const response = await fetch('/set_target');
        if (response.redirected) {
          window.location.href = response.url; // Redirect to root if redirected
        } else if (response.ok) {
          alert('Target position set successfully.');
          fetchData(); // Refresh data to update the display
        } else {
          const errorText = await response.text();
          alert(`Failed to set target position. ${errorText}`);
        }
      } catch (error) {
        console.error('Error setting target position:', error);
        alert('Error setting target position.');
      }
    }
    
    // Attach the event listener to the "Set Current Position as Target" button
    document.getElementById('setTargetBtn').addEventListener('click', setTargetPosition);
    
    // Fetch data every second
    setInterval(fetchData, 1000);
    
    // Fetch data immediately upon page load
    window.onload = fetchData;
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

// Handler for "/data" path, returns JSON data
void handleData() {
  // Calculate offset from target position
  if(currentOffsetIndex >=0 && currentOffsetIndex < numOffsets){
    offsetN = relPosN - (targetRelPosN + offsets[currentOffsetIndex].N);
    offsetE = relPosE - (targetRelPosE + offsets[currentOffsetIndex].E);
    offsetD = relPosD - (targetRelPosD + offsets[currentOffsetIndex].D);
  } else {
    offsetN = relPosN - targetRelPosN;
    offsetE = relPosE - targetRelPosE;
    offsetD = relPosD - targetRelPosD;
  }

  // Build JSON response
  String json = "{";
  json += "\"offsetN\":" + String(offsetN, 4) + ",";
  json += "\"offsetE\":" + String(offsetE, 4) + ",";
  json += "\"offsetD\":" + String(offsetD, 4) + ",";
  json += "\"ECEFX\":" + String(ECEFX, 4) + ",";
  json += "\"ECEFY\":" + String(ECEFY, 4) + ",";
  json += "\"ECEFZ\":" + String(ECEFZ, 4) + ",";
  json += "\"fixType\":\"" + fixTypeStr + "\",";
  json += "\"accuracy\":" + String(accuracy, 3) + ",";
  
  json += "\"relPosN\":" + String(relPosN, 4) + ",";
  json += "\"relPosE\":" + String(relPosE, 4) + ",";
  json += "\"relPosD\":" + String(relPosD, 4) + ",";
  json += "\"relPosHPN\":" + String(relPosHPN, 1) + ",";
  json += "\"relPosHPE\":" + String(relPosHPE, 1) + ",";
  json += "\"relPosHPD\":" + String(relPosHPD, 1) + ",";
  json += "\"accN\":" + String(accN, 4) + ",";
  json += "\"accE\":" + String(accE, 4) + ",";
  json += "\"accD\":" + String(accD, 4) + ",";
  json += "\"gnssFixOk\":" + String(gnssFixOk ? "true" : "false") + ",";
  json += "\"diffSolution\":" + String(diffSolution ? "true" : "false") + ",";
  json += "\"relPosValid\":" + String(relPosValid ? "true" : "false") + ",";
  json += "\"carrierSolutionType\":\"" + carrierSolutionType + "\",";
  json += "\"isMoving\":" + String(isMoving ? "true" : "false") + ",";
  
  json += "\"totalSats\":" + String(totalSats) + ",";
  json += "\"satsUsed\":" + String(satsUsed) + ",";
  json += "\"satsWithCorrections\":" + String(satsWithCorrections) + ",";
  json += "\"sbasCorrUsed\":" + String(sbasCorrUsed) + ",";
  json += "\"rtcmCorrUsed\":" + String(rtcmCorrUsed) + ",";
  json += "\"slasCorrUsed\":" + String(slasCorrUsed) + ",";
  json += "\"spartnCorrUsed\":" + String(spartnCorrUsed) + ",";
  json += "\"prCorrUsed\":" + String(prCorrUsed) + ",";
  json += "\"crCorrUsed\":" + String(crCorrUsed) + ",";
  json += "\"doCorrUsed\":" + String(doCorrUsed) + ",";
  
  // Add offsets list
  json += "\"offsets\":[";
  for(int i=0; i<numOffsets; i++){
    json += "{\"N\":" + String(offsets[i].N, 2) + ",\"E\":" + String(offsets[i].E, 2) + ",\"D\":" + String(offsets[i].D, 2) + "}";
    if(i < numOffsets -1){
      json += ",";
    }
  }
  json += "],";
  
  // Add survey data
  if(isSurveying){
    json += "\"isSurveying\":true,";
    json += "\"avgSurveyN\":null,";
    json += "\"avgSurveyE\":null,";
    json += "\"avgSurveyD\":null,";
    json += "\"surveyOffsetN\":null,";
    json += "\"surveyOffsetE\":null,";
    json += "\"surveyOffsetD\":null,";
    json += "\"surveyCompleted\":false";
  }
  else{
    json += "\"isSurveying\":false,";
    if(surveyCount > 0){
      json += "\"avgSurveyN\":" + String(avgSurveyN, 4) + ",";
      json += "\"avgSurveyE\":" + String(avgSurveyE, 4) + ",";
      json += "\"avgSurveyD\":" + String(avgSurveyD, 4) + ",";
      json += "\"surveyOffsetN\":" + String(surveyOffsetN, 4) + ",";
      json += "\"surveyOffsetE\":" + String(surveyOffsetE, 4) + ",";
      json += "\"surveyOffsetD\":" + String(surveyOffsetD, 4) + ",";
      json += "\"surveyCompleted\":true";
    }
    else{
      json += "\"avgSurveyN\":null,";
      json += "\"avgSurveyE\":null,";
      json += "\"avgSurveyD\":null,";
      json += "\"surveyOffsetN\":null,";
      json += "\"surveyOffsetE\":null,";
      json += "\"surveyOffsetD\":null,";
      json += "\"surveyCompleted\":false";
    }
  }
  
  json += "}";
  
  server.send(200, "application/json", json);
}

// Handler to set target position to current position
void handleSetTarget() {
  // Set target position to current RELPOSNED coordinates
  targetRelPosN = relPosN;
  targetRelPosE = relPosE;
  targetRelPosD = relPosD;

  // Respond with a redirect to root
  server.sendHeader("Location", "/");
  server.send(303); // 303 See Other
}

// Handler to move to next offset
void handleNextOffset() {
  if(numOffsets ==0){
    server.send(200, "text/plain", "No offsets available.");
    return;
  }
  currentOffsetIndex++;
  if(currentOffsetIndex >= numOffsets){
    currentOffsetIndex = 0; // Loop back to first offset
  }
  server.sendHeader("Location", "/");
  server.send(303); // Redirect to root
}

// Handler to move to previous offset
void handlePrevOffset() {
  if(numOffsets ==0){
    server.send(200, "text/plain", "No offsets available.");
    return;
  }
  currentOffsetIndex--;
  if(currentOffsetIndex <0){
    currentOffsetIndex = numOffsets -1; // Loop back to last offset
  }
  server.sendHeader("Location", "/");
  server.send(303); // Redirect to root
}

// Handler to select a specific offset
void handleSelectOffset() {
  if(server.hasArg("index")){
    int index = server.arg("index").toInt();
    if(index >=0 && index < numOffsets){
      currentOffsetIndex = index;
      server.sendHeader("Location", "/");
      server.send(303); // Redirect to root
      return;
    }
  }
  server.send(400, "text/plain", "Invalid index.");
}

// Handler to add a new offset via parameters
void handleAddOffset() {
  if(server.hasArg("N") && server.hasArg("E") && server.hasArg("D")){
    if(numOffsets >= MAX_OFFSETS){
      server.send(400, "text/plain", "Maximum number of offsets reached.");
      return;
    }
    double N = server.arg("N").toDouble();
    double E = server.arg("E").toDouble();
    double D = server.arg("D").toDouble();
    offsets[numOffsets].N = N;
    offsets[numOffsets].E = E;
    offsets[numOffsets].D = D;
    numOffsets++;
    server.sendHeader("Location", "/");
    server.send(303); // Redirect to root
    return;
  }
  server.send(400, "text/plain", "Missing parameters.");
}

// Handler to upload CSV and add offsets
void handleUploadCSV() {
  if(server.method() != HTTP_POST){
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }
  // Get the length of the request
  size_t contentLength = server.arg("plain").length();
  if(contentLength ==0){
    server.send(400, "text/plain", "No CSV data received.");
    return;
  }
  String csvData = server.arg("plain");
  
  // Parse CSV data
  int addedOffsets = 0;
  int lines = 0;
  while(csvData.length() >0 && addedOffsets < (MAX_OFFSETS - numOffsets)){
    int newlineIndex = csvData.indexOf('\n');
    String line;
    if(newlineIndex == -1){
      line = csvData;
      csvData = "";
    }
    else{
      line = csvData.substring(0, newlineIndex);
      csvData = csvData.substring(newlineIndex +1);
    }
    line.trim();
    if(line.length() ==0){
      continue; // Skip empty lines
    }
    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 +1);
    if(comma1 == -1 || comma2 == -1){
      continue; // Invalid line
    }
    String N_str = line.substring(0, comma1);
    String E_str = line.substring(comma1 +1, comma2);
    String D_str = line.substring(comma2 +1);
    double N = N_str.toDouble();
    double E = E_str.toDouble();
    double D = D_str.toDouble();
    offsets[numOffsets].N = N;
    offsets[numOffsets].E = E;
    offsets[numOffsets].D = D;
    numOffsets++;
    addedOffsets++;
    lines++;
  }
  
  String responseMessage = "Uploaded CSV with " + String(lines) + " offsets added.";
  server.send(200, "text/plain", responseMessage);
}

// Handler to start survey
void handleStartSurvey() {
  if(isSurveying){
    server.send(400, "text/plain", "Survey already in progress.");
    return;
  }
  isSurveying = true;
  surveyStartTime = millis();
  sumSurveyN = 0.0;
  sumSurveyE = 0.0;
  sumSurveyD = 0.0;
  surveyCount = 0;
  
  server.sendHeader("Location", "/");
  server.send(303); // Redirect to root
}

// Handler to stop survey
void handleStopSurvey() {
  if(!isSurveying){
    server.send(400, "text/plain", "No survey in progress.");
    return;
  }
  isSurveying = false;
  
  // Calculate average if any data was collected
  if(surveyCount > 0){
    avgSurveyN = sumSurveyN / surveyCount;
    avgSurveyE = sumSurveyE / surveyCount;
    avgSurveyD = sumSurveyD / surveyCount;
    
    // Calculate offset from target
    surveyOffsetN = avgSurveyN - (targetRelPosN + (currentOffsetIndex >=0 && currentOffsetIndex < numOffsets ? offsets[currentOffsetIndex].N : 0.0));
    surveyOffsetE = avgSurveyE - (targetRelPosE + (currentOffsetIndex >=0 && currentOffsetIndex < numOffsets ? offsets[currentOffsetIndex].E : 0.0));
    surveyOffsetD = avgSurveyD - (targetRelPosD + (currentOffsetIndex >=0 && currentOffsetIndex < numOffsets ? offsets[currentOffsetIndex].D : 0.0));
    
    // Reset accumulators
    // sumSurveyN = 0.0;
    // sumSurveyE = 0.0;
    // sumSurveyD = 0.0;
    // surveyCount = 0;
    
    Serial.print("Survey Stopped Early. Averaged Position - N: ");
    Serial.print(avgSurveyN, 4);
    Serial.print(", E: ");
    Serial.print(avgSurveyE, 4);
    Serial.print(", D: ");
    Serial.print(avgSurveyD, 4);
    Serial.print(". Offset from Target - N: ");
    Serial.print(surveyOffsetN, 4);
    Serial.print(", E: ");
    Serial.print(surveyOffsetE, 4);
    Serial.print(", D: ");
    Serial.println(surveyOffsetD, 4);
    
    // Optionally, store or process the survey results as needed
  }
  else{
    Serial.println("Survey Stopped Early. No data collected.");
  }
  
  server.sendHeader("Location", "/");
  server.send(303); // Redirect to root
}

// Handler for 404 Not Found
void handleNotFound(){
  server.send(404, "text/plain", "404: Not found");
}

// ======= Callback Functions =======

// Callback function to handle new NAV SAT summary data
void newNAVSATSummary(UBX_NAV_SAT_data_t *ubxDataStruct)
{
  // Update satellite summary variables
  totalSats = ubxDataStruct->header.numSvs;
  satsUsed = 0;
  satsWithCorrections = 0;
  sbasCorrUsed = 0;
  rtcmCorrUsed = 0;
  slasCorrUsed = 0;
  spartnCorrUsed = 0;
  prCorrUsed = 0;
  crCorrUsed = 0;
  doCorrUsed = 0;

  for (uint16_t block = 0; block < ubxDataStruct->header.numSvs; block++)
  {
    UBX_NAV_SAT_block_t *sat = &ubxDataStruct->blocks[block];

    if (sat->flags.bits.svUsed) satsUsed++;
    if (sat->flags.bits.sbasCorrUsed) { sbasCorrUsed++; }
    if (sat->flags.bits.rtcmCorrUsed) { rtcmCorrUsed++; }
    if (sat->flags.bits.slasCorrUsed) { slasCorrUsed++; }
    if (sat->flags.bits.spartnCorrUsed) { spartnCorrUsed++; }
    if (sat->flags.bits.prCorrUsed) { prCorrUsed++; }
    if (sat->flags.bits.crCorrUsed) { crCorrUsed++; }
    if (sat->flags.bits.doCorrUsed) { doCorrUsed++; }

    if (sat->flags.bits.sbasCorrUsed || sat->flags.bits.rtcmCorrUsed ||
        sat->flags.bits.slasCorrUsed || sat->flags.bits.spartnCorrUsed ||
        sat->flags.bits.prCorrUsed || sat->flags.bits.crCorrUsed ||
        sat->flags.bits.doCorrUsed)
    {
      satsWithCorrections++;
    }
  }
}

// Function to map fix type to human-readable text
String getFixTypeText(uint8_t fixType)
{
  switch (fixType)
  {
    case 0: return "NO FIX";
    case 1: return "POSSIBLE FIX";
    case 2: return "2D FIX";
    case 3: return "3D FIX";
    case 4: return "GNSS + Dead Reckoning";
    case 5: return "RTK FLOAT"; // Added RTK FLOAT
    case 6: return "RTK FIX";   // Added RTK FIX
    default: return "UNKNOWN";
  }
}

// ======= Wi-Fi Connection Function =======
void connectToWiFi(){
  if(WiFi.status() == WL_CONNECTED){
    return; // Already connected
  }

  Serial.print("Connecting to Wi-Fi network: ");
  Serial.println(ssid);
  WiFi.disconnect(true);
  WiFi.persistent(false);
  delay(1000);
  WiFi.begin(ssid, wifiPassword);

  // Non-blocking reconnection attempts handled in loop()
}

// ======= RTCM Reconnection Function =======
void reconnectRTCM(){
  if(client.connected()){
    return; // Already connected
  }

  Serial.print("Attempting to connect to RTCM server: ");
  Serial.print(rtkServer);
  Serial.print(":");
  Serial.println(rtkPort);

  if(client.connect(rtkServer, rtkPort)){
    Serial.println("Connected to RTCM server");
  }
  else{
    Serial.println("Failed to connect to RTCM server");
  }
}

// ======= Setup Function =======
void setup()
{
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Enhanced GNSS Example on RP2040 Pico W with Real-Time Web Server, Position Offsets, CSV Upload, and Survey Functionality");

  // ======= Connect to Wi-Fi =======
  connectToWiFi();

  // Wait until connected with a timeout of 30 seconds
  uint32_t startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) // 30 seconds timeout
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nFailed to connect to Wi-Fi. Entering reconnection loop.");
  }

  // ======= Start Web Server =======

  // Register HTTP handlers
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set_target", handleSetTarget);
  server.on("/next_offset", handleNextOffset);
  server.on("/prev_offset", handlePrevOffset);
  server.on("/select_offset", handleSelectOffset);
  server.on("/add_offset", handleAddOffset);
  server.on("/upload_csv", HTTP_POST, [](){
    server.send(200, "text/plain", "CSV Upload Handler");
  }, handleUploadCSV);
  server.on("/start_survey", handleStartSurvey);
  server.on("/stop_survey", handleStopSurvey);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web server started.");

  // Initialize I2C (Use default SDA and SCL pins for Pico W: GPIO 4 and GPIO 5)
  Wire.begin();

  // Initialize GNSS module
  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Set GNSS module to output UBX protocol over I2C
  myGNSS.setI2COutput(COM_TYPE_UBX);

  // Save configuration selectively (modify as needed)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

  // Set navigation frequency to 10 Hz (10 updates per second)
  myGNSS.setNavigationFrequency(10);

  // Register the NAV-SAT callback function for summary data
  myGNSS.setAutoNAVSATcallbackPtr(&newNAVSATSummary);

  // ======= Connect to RTCM Server =======
  reconnectRTCM();
}

// ======= Loop Function =======
void loop()
{
  // Handle Web Server
  server.handleClient();

  // Handle GNSS data
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();

  // ======= Handle RTCM Data =======
  if (client.connected())
  {
    // Read all available data from the server
    while (client.available())
    {
      uint8_t buf[256];
      size_t len = client.read(buf, sizeof(buf));
      if (len > 0)
      {
        // Relay RTCM data to GNSS module over I2C
        Wire.beginTransmission(GNSS_I2C_ADDRESS);
        Wire.write(buf, len);
        Wire.endTransmission();
      }
    }
  }
  else
  {
    // Attempt to reconnect RTCM server
    reconnectRTCM();
  }

  // ======= Handle Wi-Fi Reconnection =======
  unsigned long currentMillis = millis();
  if(currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL){
    lastWiFiCheck = currentMillis;
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("Wi-Fi disconnected. Attempting to reconnect...");
      connectToWiFi();
    }
    else{
      // Ensure RTCM connection is active
      if(!client.connected()){
        Serial.println("Wi-Fi reconnected. Reconnecting to RTCM server...");
        reconnectRTCM();
      }
    }
  }

  // ======= Handle ECEF Data at 10 Hz =======
  if (currentMillis - lastECEFTimestamp >= ECEF_INTERVAL)
  {
    lastECEFTimestamp = currentMillis;

    // Collect the position data
    int32_t tmp_ECEFX = myGNSS.getHighResECEFX();
    int8_t tmp_ECEFXHp = myGNSS.getHighResECEFXHp();
    int32_t tmp_ECEFY = myGNSS.getHighResECEFY();
    int8_t tmp_ECEFYHp = myGNSS.getHighResECEFYHp();
    int32_t tmp_ECEFZ = myGNSS.getHighResECEFZ();
    int8_t tmp_ECEFZHp = myGNSS.getHighResECEFZHp();
    uint32_t tmp_accuracy = myGNSS.getPositionAccuracy();

    // Assemble the high precision coordinates
    ECEFX = ((double)tmp_ECEFX) / 100.0; // Convert from cm to m
    ECEFX += ((double)tmp_ECEFXHp) / 10000.0;  // Add high resolution component
    ECEFY = ((double)tmp_ECEFY) / 100.0;
    ECEFY += ((double)tmp_ECEFYHp) / 10000.0;
    ECEFZ = ((double)tmp_ECEFZ) / 100.0;
    ECEFZ += ((double)tmp_ECEFZHp) / 10000.0;

    // Get fix type
    uint8_t fixType = myGNSS.getFixType();
    fixTypeStr = getFixTypeText(fixType);

    // Convert accuracy to meters
    accuracy = (float)tmp_accuracy / 1000.0; // mm to m

    // Get RELPOSNED data if available
    if (myGNSS.getRELPOSNED() == true)
    {
      relPosN = myGNSS.getRelPosN();
      relPosE = myGNSS.getRelPosE();
      relPosD = myGNSS.getRelPosD();

      relPosHPN = (float)myGNSS.packetUBXNAVRELPOSNED->data.relPosHPN / 10.0; // mm
      relPosHPE = (float)myGNSS.packetUBXNAVRELPOSNED->data.relPosHPE / 10.0; // mm
      relPosHPD = (float)myGNSS.packetUBXNAVRELPOSNED->data.relPosHPD / 10.0; // mm

      accN = myGNSS.getRelPosAccN();
      accE = myGNSS.getRelPosAccE();
      accD = myGNSS.getRelPosAccD();

      gnssFixOk = myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.gnssFixOK;
      diffSolution = myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.diffSoln;
      relPosValid = myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.relPosValid;

      // Carrier Solution Type
      uint8_t carrSoln = myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.carrSoln;
      switch (carrSoln)
      {
        case 0: carrierSolutionType = "None"; break;
        case 1: carrierSolutionType = "Float"; break;
        case 2: carrierSolutionType = "Fixed"; break;
        default: carrierSolutionType = "Unknown"; break;
      }

      isMoving = myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.isMoving;

      // If surveying, accumulate data
      if(isSurveying){
        sumSurveyN += relPosN;
        sumSurveyE += relPosE;
        sumSurveyD += relPosD;
        surveyCount++;
      }
    }

    // Print the ECEF coordinates with 4 decimal places (0.1mm)
    Serial.print("ECEF Coordinates - X (m): ");
    Serial.print(ECEFX, 4);
    Serial.print(", Y (m): ");
    Serial.print(ECEFY, 4);
    Serial.print(", Z (m): ");
    Serial.print(ECEFZ, 4);

    // Print fix type and accuracy
    Serial.print(", Fix Type: ");
    Serial.print(fixTypeStr);
    Serial.print(", Accuracy (m): ");
    Serial.println(accuracy, 3);

    // Check if survey duration has elapsed
    if(isSurveying && (millis() - surveyStartTime >= surveyDuration)){
      // Stop the survey
      isSurveying = false;
      
      // Calculate average
      if(surveyCount > 0){
        avgSurveyN = sumSurveyN / surveyCount;
        avgSurveyE = sumSurveyE / surveyCount;
        avgSurveyD = sumSurveyD / surveyCount;

        // Calculate offset from target
        surveyOffsetN = avgSurveyN - (targetRelPosN + (currentOffsetIndex >=0 && currentOffsetIndex < numOffsets ? offsets[currentOffsetIndex].N : 0.0));
        surveyOffsetE = avgSurveyE - (targetRelPosE + (currentOffsetIndex >=0 && currentOffsetIndex < numOffsets ? offsets[currentOffsetIndex].E : 0.0));
        surveyOffsetD = avgSurveyD - (targetRelPosD + (currentOffsetIndex >=0 && currentOffsetIndex < numOffsets ? offsets[currentOffsetIndex].D : 0.0));

        Serial.print("Survey Completed. Averaged Position - N: ");
        Serial.print(avgSurveyN, 4);
        Serial.print(", E: ");
        Serial.print(avgSurveyE, 4);
        Serial.print(", D: ");
        Serial.print(avgSurveyD, 4);
        Serial.print(". Offset from Target - N: ");
        Serial.print(surveyOffsetN, 4);
        Serial.print(", E: ");
        Serial.print(surveyOffsetE, 4);
        Serial.print(", D: ");
        Serial.println(surveyOffsetD, 4);

        // Reset survey variables
        // surveyStartTime = 0;
        // sumSurveyN = 0.0;
        // sumSurveyE = 0.0;
        // sumSurveyD = 0.0;
        // surveyCount = 0;
      }
      else{
        // No data collected
        Serial.println("Survey Completed. No data collected.");
        surveyStartTime = 0;
      }
    }
  }

  // ======= Handle RELPOSNED Data =======
  // Already updated above if available

  // ======= Additional Housekeeping =======
  // (Any additional tasks can be added here)

  // No blocking delay to ensure loop runs smoothly
}