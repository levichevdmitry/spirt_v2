var o=new XMLHttpRequest();
var press_g, power_g, water_g;
var params; // contain all get params from device
var pChart;
var tChart;
var tempr_data = {
        labels: ["+", "+"],
        datasets: [{
            label: 'Температура в кубе, С',
            data: [0.0, 1.0],
            pointRadius: 0,
            pointBorderWidth: 0,
            fill: false,
            borderColor: 'rgba(250, 0, 0, 0.5)',
            backgroundColor: 'rgba(250, 0, 0, 0.1)'
        },
        {
            label: 'Температура колоны низ, С',
            data: [0.0, 1.0],
            pointRadius: 0,
            pointBorderWidth: 0,
            fill: false,
            borderColor: 'rgba(0, 250, 0, 0.5)',
            backgroundColor: 'rgba(0, 250, 0, 0.1)' 
        },
        {
            label: 'Температура колоны верх, С',
            data: [0.0, 1.0],
            pointRadius: 0,
            pointBorderWidth: 0,
            fill: false,
            borderColor: 'rgba(0, 0, 250, 0.5)',
            backgroundColor: 'rgba(0, 0, 250, 0.1)' 
        }
        ]
    };

var press_data = {
        labels: ["+", "+"],
        datasets: [{
            label: 'Давление в кубе, мм рт. ст.',
            data: [0.0, 1.0],
            pointRadius: 0,
            pointBorderWidth: 0,
            fill: false,
            borderColor: 'rgba(0, 250, 0, 0.5)',
            backgroundColor: 'rgba(0, 250, 0, 0.1)'
        },
        {
            label: 'Мощность, кВт',
            data: [0.0, 1.0],
            pointRadius: 0,
            pointBorderWidth: 0,
            fill: false,
            borderColor: 'rgba(0, 0, 250, 0.5)',
            backgroundColor: 'rgba(0, 0, 250, 0.1)'
        }]
    };

function loadChart(){
var t_ctx = document.getElementById("tChart").getContext('2d');
 tChart = new Chart(t_ctx, {
    type: 'line',
    data: tempr_data,
    options: {
        scales: {
            yAxes: [{
                ticks: {
                    beginAtZero:true
                }
            }]
        }
    }
});
var p_ctx = document.getElementById("pChart").getContext('2d');
 pChart = new Chart(p_ctx, {
    type: 'line',
    data: press_data,
    options: {
        scales: {
            yAxes: [{
                ticks: {
                    beginAtZero:true
                }
            }]
        }
    }
});
}

function addChartData() {
    if (tempr_data.datasets.length > 0) {
        tempr_data.labels.push("+");
        tempr_data.datasets.forEach(function(dataset, datasetIndex) {
            if (datasetIndex == 0) {
                dataset.data.push(params.tkub);
            } else if (datasetIndex == 1){
                dataset.data.push(params.tkolonan);
            } else if (datasetIndex == 2){
                dataset.data.push(params.tkolonav);
            }
        });
        if (tempr_data.labels.length > 60) { // 60 sec
            tempr_data.labels.splice(0, 1); // remove the label first
            tempr_data.datasets.forEach(function(dataset) {
                dataset.data.splice(0, 1);
            });
        }
        window.tChart.update();
    }

    if (press_data.datasets.length > 0) {
        press_data.labels.push("+");
        press_data.datasets.forEach(function(dataset, datasetIndex) {
            if (datasetIndex == 0) {
                dataset.data.push(params.pkub);
            } else if (datasetIndex == 1){
                dataset.data.push(params.power);
            } 
        });
        if (press_data.labels.length > 60) { // 60 sec
            press_data.labels.splice(0, 1); // remove the label first
            press_data.datasets.forEach(function(dataset) {
                dataset.data.splice(0, 1);
            });
        }
        window.pChart.update();
    }
}

document.addEventListener("DOMContentLoaded", function(event) {
        press_g = new JustGage({
            id: "press_g",
            value: 0.0,
            min: 0.0,
            max: 50.0,
            relativeGaugeSize: true,
            title: "Давление в кубе, мм рт. ст",
            decimals: 1,
            gaugeWidthScale: 0.5,
            gaugeColor: "#fff",
            donut: true
        });

         power_g = new JustGage({
            id: "power_g",
            value: 0.0,
            min: 0.0,
            max: 6.0,
            decimals: 1,
            gaugeWidthScale: 0.5,
            gaugeColor: "#fff",
            title: "Мощность, кВт",
            relativeGaugeSize: true,
            donut: true
        });
        
         water_g = new JustGage({
            id: "water_g",
            value: 0.0,
            min: 0.0,
            max: 5.0,
            decimals: 1,
            gaugeWidthScale: 0.5,
            gaugeColor: "#fff",
            title: "Расход, л/мин",
            relativeGaugeSize: true,
            donut: true
        });
  loadChart();
});

function dR() {
    o.open("GET","/data.dat?r="+Math.random(), 1);
    o.onload=function(){
      console.dir(this.responseText);
      params = JSON.parse(this.responseText);
      
      for (var k in  params) {
          try {
            document.getElementById(k).innerText=params[k];
          } catch (e) {}
      }
      press_g.refresh(params.pkub);
      power_g.refresh(params.power);
      water_g.refresh(params.fcoolant);
      addChartData();    
      setTimeout("dR()", 1000);
   } 
   o.send();
  }