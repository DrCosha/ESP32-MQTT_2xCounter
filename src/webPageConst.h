/*
************************************************************************
*   Включаемый файл с константыми строками и флагами для генерации WEB
*              страниц для контроллера подсчёта импульсов
*                        (с) 2024, by Dr@Cosha
************************************************************************
*/
      
// сюда вынесены все константные строки для генерации WEB страниц

String CSW_PAGE_TITLE = R"=====(<!DOCTYPE html><html lang="en" class=""><head><meta charset="utf-8"> <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>)=====";
String CSW_PAGE_STYLE = R"=====(<style> div,fieldset,input,select{padding:5px;font-size:1em;} fieldset{background:#4f4f4f;} p{margin:0.5em 0;}
 input{width:100%;box-sizing:border-box;-webkit-box-sizing:border-box;-moz-box-sizing:border-box;background:#dddddd;color:#000000;}
 input[type=checkbox], input[type=radio]{width:1em;margin-right:6px;vertical-align:-1px;} input[type=range]{width:99%;} select{width:100%;background:#dddddd;color:#000000;}
 textarea{resize:vertical;width:98%;height:318px;padding:5px;overflow:auto;background:#1f1f1f;color:#65c115;} body{text-align:center;font-family:verdana,sans-serif;background:#252525;}
 td{padding:0px;} button{border:0;border-radius:0.3rem;background:#1fa3ec;color:#faffff;line-height:2.4rem;font-size:1.2rem;width:100%;-webkit-transition-duration:0.4s;transition-duration:0.4s;cursor:pointer;}
 button:hover{background:#0e70a4;} .bred{background:#d43535;}.bred:hover{background:#931f1f;}.bgrn{background:#47c266;}.bgrn:hover{background:#5aaf6f;} a{color:#1fa3ec;text-decoration:none;}
 .p{float:left;text-align:left;}.q{float:right;text-align:right;}.r{border-radius:0.3em;padding:2px;margin:6px 2px;} #blink {-webkit-animation: blink 2s linear infinite;animation: blink 2s linear infinite;font-weight: bold;color: color: #F00;}
 @-webkit-keyframes blink {0%{ color: #ff0000; }50%{ color: #1f1f1f; }100%{ color: #ff0000;}} @keyframes blink{0%{color:#ff0000;}50%{color:#1f1f1f;}100%{color:#ff0000;}}</style>)=====";
String CSW_PAGE_FOOTER = R"=====(</form><p></p><div style="text-align:right;font-size:11px;"><hr><a style="color:#aaa;">(c)Dr.Cosha 2024 (based on design by Theo Arends)</a></div></div></body></html>)=====";

bool f_ApplayChanges = false;                   // флаг для генерации страницы примененных изменений