var o=new XMLHttpRequest();
function dR() {
    o.open("GET","/data.dat?r="+Math.random(),1);
    o.onload=function(){
      console.dir(this.responseText);
      var params = JSON.parse(this.responseText, function (k, v) {
          try {
            document.getElementById(k).innerText=v;
            if (k === "tkolonan") {
              console.dir("g3 refresh");
              g3.refresh(getRandomInt(0, 100));
            }
          } catch (e) {

          }
      });
     setTimeout("dR()", 1000);
   } 
   o.send();
  }