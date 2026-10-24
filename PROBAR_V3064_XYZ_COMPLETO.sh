#!/usr/bin/env bash
set -Eeuo pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

BASE="$HOME/Proyecto_Lidar/pruebas/V3064_XYZ/V30_6_4_PROPUESTA_PASILLO/CODIGO"
XYZ="$HOME/Proyecto_Lidar/data/nube_puntos_221026_1911.xyz"
OUT="$HOME/Proyecto_Lidar/pruebas/V3064_FULL_$(date +%Y%m%d_%H%M%S)"
RUNNER="$OUT/run_v3064.py"

EXP="$BASE/exporter_node_V30_6_4_PASILLO_CANDIDATO.py"
CAD="$BASE/cad_reconstructor_V30_6_4_PASILLO_CANDIDATO.py"

for f in "$XYZ" "$EXP" "$CAD"; do
  test -s "$f" || { echo "ERROR: falta $f"; exit 2; }
done
mkdir -p "$OUT"
export V3064_OUT="$OUT" V3064_XYZ="$XYZ" V3064_EXP="$EXP" V3064_CAD="$CAD"
export LIDAR_CAD_REFERENCE_PLANTA=""

cat > "$RUNNER" <<'PY'
from pathlib import Path
import os, sys, json, shutil, math, importlib.util
import numpy as np
import ezdxf
import rclpy

OUT=Path(os.environ['V3064_OUT'])
XYZ=Path(os.environ['V3064_XYZ'])
EXP=Path(os.environ['V3064_EXP'])
CAD=Path(os.environ['V3064_CAD'])

spec=importlib.util.spec_from_file_location('v3064_exporter', EXP)
mod=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=mod
spec.loader.exec_module(mod)
mod.DATA_DIR=OUT
mod.CAD_RECONSTRUCTOR=CAD

def make_plan_dwg(node, src, dst):
    di=ezdxf.readfile(str(src))
    do=ezdxf.new('R2000')
    do.header['$INSUNITS']=di.header.get('$INSUNITS',6)
    for lay in di.layers:
        if lay.dxf.name not in do.layers:
            do.layers.add(lay.dxf.name)
    m=do.modelspace()
    for e in di.modelspace():
        typ=e.dxftype(); props={'layer':e.dxf.layer}
        if typ=='LINE':
            m.add_line(tuple(e.dxf.start),tuple(e.dxf.end),dxfattribs=props)
        elif typ=='LWPOLYLINE':
            pts=[(float(q[0]),float(q[1]),float(e.dxf.elevation or 0)) for q in e.get_points('xy')]
            for a,b in zip(pts,pts[1:]): m.add_line(a,b,dxfattribs=props)
            if e.closed and len(pts)>2: m.add_line(pts[-1],pts[0],dxfattribs=props)
        elif typ=='POLYLINE':
            pts=[tuple(v.dxf.location) for v in e.vertices]
            for a,b in zip(pts,pts[1:]): m.add_line(a,b,dxfattribs=props)
            if e.is_closed and len(pts)>2: m.add_line(pts[-1],pts[0],dxfattribs=props)
        elif typ in ('CIRCLE','ARC'):
            cx,cy=float(e.dxf.center.x),float(e.dxf.center.y); r=float(e.dxf.radius)
            a0=0.0 if typ=='CIRCLE' else math.radians(float(e.dxf.start_angle))
            a1=2*math.pi if typ=='CIRCLE' else math.radians(float(e.dxf.end_angle))
            if typ=='ARC' and a1<=a0: a1+=2*math.pi
            n=24 if typ=='CIRCLE' else max(8,int((a1-a0)/(2*math.pi)*24))
            pts=[(cx+r*math.cos(a0+(a1-a0)*i/n),cy+r*math.sin(a0+(a1-a0)*i/n),0.0) for i in range(n+1)]
            for a,b in zip(pts,pts[1:]): m.add_line(a,b,dxfattribs=props)
        elif typ in ('TEXT','MTEXT'):
            pass
        else:
            print('AVISO_DWG_OMITIDO',typ,e.dxf.layer)
    compat=OUT/'_planta_compatible.dxf'
    do.saveas(str(compat))
    node.convert_dxf_to_dwg(compat,dst)
    node.normalize_dwgs_qcad(dst)

rclpy.init()
node=mod.ExporterNode()
try:
    print('CARGANDO_XYZ',XYZ)
    pts=np.loadtxt(XYZ,dtype=np.float64)
    pts=pts[np.isfinite(pts).all(axis=1)]
    print('PUNTOS',len(pts))
    mark='221026_1911_V3064TEST'
    paths=node.paths_for_mark(mark)
    shutil.copy2(XYZ,paths['xyz'])

    print('1/6 PLANTA')
    geom=node.generate_plan_dxf(pts,paths['planta'])
    diag=node._analyze_v30(pts,geom)
    geom['v30_diag']=diag

    print('2/6 MEDIDAS')
    meas=node.generate_measures(pts,geom,paths['medidas'])
    node._append_v30_measures(paths['medidas'],diag)

    print('3/6 STL')
    node.generate_stl(geom,meas,paths['stl'])

    print('4/6 DWG_PLANTA')
    make_plan_dwg(node,paths['planta'],paths['planta_dwg'])

    print('5/6 STEP_DWG_3D')
    cad_report=node.generate_cad_outputs(mark,paths,height_mm=(float(geom['ceiling'])-float(geom['floor']))*1000.0)

    passages=geom.get('open_passages') or []
    d=ezdxf.readfile(str(paths['planta']))
    access=list(d.modelspace().query('*[layer=="ACCESOS_ABIERTOS"]'))
    access_lines=[e for e in access if e.dxftype()=='LINE']
    access_circles=[e for e in access if e.dxftype()=='CIRCLE']

    report={
      'mark':mark,'points':int(len(pts)),
      'room_closed':bool(geom.get('room_closed')),
      'reconstruction_source':geom.get('reconstruction_source'),
      'floor_m':float(geom['floor']),'ceiling_m':float(geom['ceiling']),
      'height_m':float(geom['ceiling']-geom['floor']),
      'multilevel_segments':len(geom.get('multilevel_segments') or []),
      'open_passages':passages,
      'access_layer_entities':len(access),
      'access_layer_lines':len(access_lines),
      'access_layer_circles':len(access_circles),
      'cad':cad_report,
    }
    paths['diag'].write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')

    seven=[paths[k] for k in ('medidas','xyz','planta','planta_dwg','stl','solido_dwg','solido_step')]
    bad=[str(x) for x in seven if not x.is_file() or x.stat().st_size<=0]
    if bad: raise RuntimeError('SALIDAS_FALTANTES: '+', '.join(bad))
    if not passages: raise RuntimeError('NO_SE_DETECTO_PASO_ABIERTO')
    if access_lines: raise RuntimeError('ERROR: existe LINEA de cierre en ACCESOS_ABIERTOS')
    if len(access_circles)<2: raise RuntimeError('Faltan marcas de extremos del paso abierto')

    print('\n=== PRUEBA INTEGRAL V30.6.4 SUPERADA ===')
    print('Recinto cerrado:',bool(geom.get('room_closed')))
    print('Fuente:',geom.get('reconstruction_source'))
    print('Suelo/techo/altura:',round(float(geom['floor']),3),round(float(geom['ceiling']),3),round(float(geom['ceiling']-geom['floor']),3),'m')
    print('Muros multialtura:',len(geom.get('multilevel_segments') or []))
    print('Pasos abiertos verificados:',len(passages))
    for i,p in enumerate(passages,1):
        print('PASO',i,'ancho=',round(float(p['width_m']),3),'m','rayos=',p['beam_diversity'],'prof=',round(float(p['median_clear_depth_m']),2),'m')
    print('7 ARCHIVOS OK:')
    for x in seven: print(' ',x.name,x.stat().st_size,'bytes')
    print('RESULTADOS:',OUT)
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
PY

python3 -u "$RUNNER" 2>&1 | tee "$OUT/PRUEBA_INTEGRAL.log"
echo
echo "Informe: $OUT/PRUEBA_INTEGRAL.log"
echo "Resultados: $OUT"
