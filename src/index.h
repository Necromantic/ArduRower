const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<style>
.card{
    max-width: 768px;
    min-height: 250px;
    background: #02b875;
    padding: 30px;
    box-sizing: border-box;
    color: #FFF;
    margin:20px;
    box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
}
</style>
<body>

<div class="card">
  <h1>Stroke Rate: <span class="rowingData" id="strokeRate">0</span></h1><br>
  <h1>stroke Count: <span class="rowingData" id="strokeCount">0</span></h1><br>
  <h1>Average Stroke Rate Value: <span class="rowingData" id="averageStokeRate">0</span></h1><br>
  <h1>Total Distance: <span class="rowingData" id="totalDistance">0</span></h1><br>
  <h1>Instantaneous Pace: <span class="rowingData" id="instantaneousPace">0</span></h1><br>
  <h1>Average Pace: <span class="rowingData" id="averagePace">0</span></h1><br>
  <h1>Instantaneous Power: <span class="rowingData" id="instantaneousPower">0</span></h1><br>
  <h1>Average Power: <span class="rowingData" id="averagePower">0</span></h1><br>
  <h1>Elapsed Time: <span class="rowingData" id="elapsedTime">0</span></h1><br>
</div>
<script>

setInterval(function() {
  getData();
}, 500);

function getData() {
  var ajaxRequest = new XMLHttpRequest();
  ajaxRequest.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var jsonData = JSON.parse(ajaxRequest.responseText)

      var rowingData = document.getElementsByClassName('rowingData');
      rowingData.item(0).textContent = jsonData.strokeRate;
      rowingData.item(1).textContent = jsonData.strokeCount;
      rowingData.item(2).textContent = jsonData.averageStokeRate;
      rowingData.item(3).textContent = jsonData.totalDistance;
      rowingData.item(4).textContent = jsonData.instantaneousPace;
      rowingData.item(5).textContent = jsonData.averagePace;
      rowingData.item(6).textContent = jsonData.instantaneousPower;
      rowingData.item(7).textContent = jsonData.averagePower;
      rowingData.item(8).textContent = jsonData.elapsedTime;
    }
  };
  ajaxRequest.open("GET", "getData", true);
  ajaxRequest.send();
}
</script>
</body>
</html>
)=====";