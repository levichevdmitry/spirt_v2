var o=new XMLHttpRequest();
//var ver = new XMLHttpRequest();
var webversion;
var press_g, power_g, water_g;
var params; // contain all get params from device
var pChart;
var tChart;
var pwrChart;

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
        }]
    };

var pwr_data = {
        labels: ["+", "+"],
        datasets: [{
            label: 'Мощность, кВт',
            data: [0.0, 1.0],
            pointRadius: 0,
            pointBorderWidth: 0,
            fill: false,
            borderColor: 'rgba(255, 69, 0, 0.5)',
            backgroundColor: 'rgba(255, 69, 0, 0.1)'
        }]
    };

var mode_names = [
"Ничего не запущено", 
"Инициализация датчиков температуры",
"Чистка паром колонны",
"Дистилляция",
"Ректификация",
"Тестирование датчиков",
"Калибровка клапана"
];

var smode_names = [
[
"Ожидание..."
],
[
"Инициализация"
],
[
"Разогрев",
"Чистка",
"Чистка клапанов",
"Чистка завершена"
],
[
"Подготовка",
"Преднагрев",
"Разогрев",
"Отбор голов",
"Работа на себя",
"Отбор тела",
"Отбор хвостов",
"Окончание процесса",
"Пауза отбора",
"Остановка"
],
[
"Подготовка",
"Преднагрев",
"Разогрев. Стабилизация колонны",
"Работа на себя ",
"Отбор голов",
"Отбор подголовников",
"Работа на себя",
"Отбор тела",
"Отбор хвостов",
"Окончание процесса",
"Пауза отбора",
"Остановка"
],
[
"Тестирование..."
],
[
"Подготовка",
"Калибровка",
"Сохранение",
"Остановка",
"Запуск нагрева"
]
];

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
var pwr_ctx = document.getElementById("pwrChart").getContext('2d');
 pwrChart = new Chart(pwr_ctx, {
    type: 'line',
    data: pwr_data,
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
                dataset.data.push(params.t_kub);
            } else if (datasetIndex == 1){
                dataset.data.push(params.t_koln);
            } else if (datasetIndex == 2){
                dataset.data.push(params.t_kolv);
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
            //if (datasetIndex == 0) {
                dataset.data.push(params.p_kub);
            //} 
        });
        if (press_data.labels.length > 60) { // 60 sec
            press_data.labels.splice(0, 1); // remove the label first
            press_data.datasets.forEach(function(dataset) {
                dataset.data.splice(0, 1);
            });
        }
        window.pChart.update();
    }

    if (pwr_data.datasets.length > 0) {
        pwr_data.labels.push("+");
        pwr_data.datasets.forEach(function(dataset, datasetIndex) {
            //if (datasetIndex == 1){
                dataset.data.push(params.pwr);
            //} 
        });
        if (pwr_data.labels.length > 60) { // 60 sec
            pwr_data.labels.splice(0, 1); // remove the label first
            pwr_data.datasets.forEach(function(dataset) {
                dataset.data.splice(0, 1);
            });
        }
        window.pwrChart.update();
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

function setStatus(){

    document.getElementById("mode_descr").innerText = mode_names[params.mode];
    if (params.smode != 255) {
        document.getElementById("status").innerText = smode_names[params.mode][params.smode];
    } else {
        document.getElementById("status").innerText = "Аварийная остановка!";
    }
    if ((params.mode == 3) && (params.smode == 4 || params.smode == 7)) { // Дистилляция
        document.getElementById("t_elapse").innerText = params.t_elapsed + " мин";
    } else if ((params.mode == 4) && (params.smode == 2 || params.smode == 3 || params.smode == 6 || params.smode == 9)) { // Ректификация
        document.getElementById("t_elapse").innerText = params.t_elapsed + " мин";
    } else {
        document.getElementById("t_elapse").innerText = "";
    }
    // decode ETA word
    if (params.eta & 0x02) {
        document.getElementById("t_pwr_mod_state").innerText = "Перегрев";
        document.getElementById("t_pwr_mod_state").className = "status_red";
    } else {
        document.getElementById("t_pwr_mod_state").innerText = "Норма";
        document.getElementById("t_pwr_mod_state").className = "status";
    }
    if (params.eta & 0x04) {
        document.getElementById("t_sen_oj_state").innerText = "Подключен";
        document.getElementById("t_sen_oj_state").className = "status";
    } else {
        document.getElementById("t_sen_oj_state").innerText = "Отключен";
        document.getElementById("t_sen_oj_state").className = "status_red";
    }
    if (params.eta & 0x08) {
        document.getElementById("l_sen_state").innerText = "Норма";
    } else {
        document.getElementById("l_sen_state").innerText = "Низкий";
    }
    if (params.eta & 0x40) {
        document.getElementById("pump_oj_state").innerText = "Вкл.";
    } else {
        document.getElementById("pump_oj_state").innerText = "Выкл.";
    }
    if (params.eta & 0x80) {
        document.getElementById("fan_oj_state").innerText = "Вкл.";
    } else {
        document.getElementById("fan_oj_state").innerText = "Выкл.";
    }

    if (params.alcopdk) {
        document.getElementById("alcopdk").innerText = "Высокая";
        document.getElementById("alcopdk").className = "status_red";
    } else {
        document.getElementById("alcopdk").innerText = "Норма";
        document.getElementById("alcopdk").className = "status";
    }
}

function load(){
    o.open("GET","/version.dat?r="+Math.random(), 1);
    o.onload=function(){
      console.dir(this.responseText);
      try {
        webversion = JSON.parse(this.responseText);
      } catch (e) {
        load_data();
      }
      for (var k in  webversion) {
          try {
            document.getElementById(k).innerText=webversion[k];
          } catch (e) {
            console.dir(e);
          }
      }
      load_data();
   } 
   o.send();
   
}

function load_data() {
    o.open("GET","/data.dat?r="+Math.random(), 1);
    //o.open("GET","/data.php?r="+Math.random(), 1);
    o.onload=function(){
      console.dir(this.responseText);
      try {
        params = JSON.parse(this.responseText);
      } catch (e) {
        setTimeout("load_data()", 1000);
      }
      for (var k in  params) {
          try {
            document.getElementById(k).innerText=params[k];
          } catch (e) {}
      }
      press_g.refresh(params.p_kub);
      power_g.refresh(params.pwr);
      water_g.refresh(params.f_oj);
      addChartData();
      setStatus();    
      setTimeout("load_data()", 1000);
   } 
   o.send();
  }