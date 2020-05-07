#Author-Mohamed Mashaly
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback
import math
handlers=[]

def calcualtegear(m,n,al,s,fw,f,tw,fi,hd,kw,kh,oh,ch):
    x=[]
    y=[]
    x2=[]
    y2=[]
    xh=[]
    yh=[]
    zh=[]
    dp=n*m
    if str(ch)== 'Taper Gear' or str(ch)== 'Helix Taper Gear':
        f=((dp/2)*math.tan(math.pi*0.5-f))/((dp/2)*math.tan(math.pi*0.5-f)-fw)
    db=dp*math.cos(al)
    da=dp+2*m
    dd=dp-2*1.25*m
    t=0
    a=-1
    delt=0.1
    while db*0.5*(math.cos(t)+t*math.sin(t)) <= (da/1.9)*(math.cos(t/10)) or db*0.5*(math.cos(t)+t*math.sin(t)) >= (da/1.9)*(math.cos(t/10))*1.001:
        if db*0.5*(math.cos(t)+t*math.sin(t)) <= (da/1.9)*(math.cos(t/10)):
            if a==1:
                delt=delt/2
            t=t+delt
            a=-1
        if db*0.5*(math.cos(t)+t*math.sin(t)) >= (da/1.9)*(math.cos(t/10))*1.001: 
            if a==-1:
                delt=delt/2
            t=t-delt
            a=1
                                                                                    
    t1=t
    b=math.pi/(2*n)+al-(((dp**2-db**2)**0.5)/db)
    t=0
    while t<= t1:
        t=t+t1/s
        x.append(db*0.5*(math.cos(t)+t*math.sin(t)))
        y.append(db*0.5*(math.sin(t)-t*math.cos(t)))
        x2.append(db*0.5*(math.cos(-t-2*b)-t*math.sin(-t-2*b)))
        y2.append(db*0.5*(math.sin(-t-2*b)+t*math.cos(-t-2*b)))
    if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear' or str(ch)== 'Internal Helix Gear':
        t=0
        k=0
        if str(ch)== 'Helix Taper Gear':
            while t <= tw:
                k=t*fw/tw
                xh.append(((1+k*(1/f-1)/fw)*da/(1.9))*math.cos(-t))
                yh.append(((1+k*(1/f-1)/fw)*da/(1.9))*math.sin(-t))
                zh.append(t*fw/tw)
                t=t+tw/s
                k=t*fw/tw
            xh.append(((1+k*(1/f-1)/fw)*da/(1.9))*math.cos(-t))
            yh.append(((1+k*(1/f-1)/fw)*da/(1.9))*math.sin(-t))
            zh.append(t*fw/tw)
        if str(ch)== 'Helix Gear'or str(ch)== 'Internal Helix Gear':
            while t <= tw:
                k=t*fw/tw
                xh.append((da/(1.9))*math.cos(-t))
                yh.append((da/(1.9))*math.sin(-t))
                zh.append(t*fw/tw)
                t=t+tw/s
            xh.append((da/(1.9))*math.cos(-t))
            yh.append((da/(1.9))*math.sin(-t))
            zh.append(t*fw/tw)
    if str(ch)== 'Taper Gear' or str(ch)== 'Helix Taper Gear' or str(ch)== 'Helix Gear' or str(ch)== 'Spur Gear':
        CreateTaberHelixGear(x,y,x2,y2,dd,da,fw,n,f,tw,xh,yh,zh,fi,hd,kw,kh,ch)
    if str(ch)== 'Internal Helix Gear' or str(ch)== 'Internal Spur Gear':
        CreateExternalHelixGear(x,y,x2,y2,dd,da,fw,n,tw,xh,yh,zh,fi,oh,ch)
    


def CreateTaberHelixGear(x,y,x2,y2,dd,da,fw,n,f,tw,xh,yh,zh,fi,hd,kw,kh,ch):
        app = adsk.core.Application.get()
        design=adsk.fusion.Design.cast(app.activeProduct)
        root=design.rootComponent
        occ=root.occurrences
        com=occ.addNewComponent(adsk.core.Matrix3D.create())
        com.component.name='Gear'
        #-----------------------------------------------------------------------
        sk1=com.component.sketches.add(com.component.xYConstructionPlane)
        sk1.sketchCurves.sketchCircles.addByCenterRadius(sk1.origin,da/2)
        #-----------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear'or str(ch)== 'Taper Gear':
            pls=com.component.constructionPlanes.createInput()
            pls.setByOffset(com.component.xYConstructionPlane,adsk.core.ValueInput.createByReal(fw))
            plane2=com.component.constructionPlanes.add(pls)
        #-----------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear'or str(ch)== 'Taper Gear':
            sk11=com.component.sketches.add(plane2)
            if str(ch)== 'Helix Taper Gear'or str(ch)== 'Taper Gear':
                sk11.sketchCurves.sketchCircles.addByCenterRadius(sk11.originPoint,da/(2*f))
            if str(ch)== 'Helix Gear':
                sk11.sketchCurves.sketchCircles.addByCenterRadius(sk11.originPoint,da/(2))
            sk11.isVisible=False
        #_________________________________________________________________________
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear'or str(ch)== 'Taper Gear':
            exinput=com.component.features.loftFeatures.createInput(3)
            exinput.loftSections.add(sk1.profiles.item(0))
            exinput.loftSections.add(sk11.profiles.item(0))
            com.component.features.loftFeatures.add(exinput)
        if str(ch)== 'Spur Gear':
            com.component.features.extrudeFeatures.addSimple(sk1.profiles.item(0),adsk.core.ValueInput.createByReal(fw),3)
        #_________________________________________________________________________
        points1=adsk.core.ObjectCollection.create()
        points2=adsk.core.ObjectCollection.create()
        for i in range(0,len(x)):
            points1.add(adsk.core.Point3D.create(x[i],y[i],0))
            points2.add(adsk.core.Point3D.create(x2[i],y2[i],0))
        #------------------------------------------------------------------------
        sk2=com.component.sketches.add(com.component.xYConstructionPlane)
        sk2.sketchCurves.sketchFittedSplines.add(points1)
        sk2.sketchCurves.sketchFittedSplines.add(points2)
        sk2.sketchCurves.sketchCircles.addByCenterRadius(sk2.origin,da/1.9)
        sk2.sketchCurves.sketchCircles.addByCenterRadius(sk2.origin,dd/2)
        sk2.sketchCurves.sketchLines.addByTwoPoints(sk2.origin,points1.item(0))
        sk2.sketchCurves.sketchLines.addByTwoPoints(sk2.origin,points2.item(0))
        sk2.arePointsShown=False
        #________________________________________________________________________
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear' or str(ch)== 'Taper Gear':
            points1=adsk.core.ObjectCollection.create()
            points2=adsk.core.ObjectCollection.create()
            xr=[]
            yr=[]
            xr2=[]
            yr2=[]
            for i in range(0,len(x)):
                if str(ch)== 'Helix Gear':
                    xr.append(math.cos(tw)*x[i]+math.sin(tw)*y[i])
                    yr.append(-math.sin(tw)*x[i]+math.cos(tw)*y[i])
                    xr2.append(math.cos(tw)*x2[i]+math.sin(tw)*y2[i])
                    yr2.append(-math.sin(tw)*x2[i]+math.cos(tw)*y2[i])
                    points1.add(adsk.core.Point3D.create(xr[i],yr[i],0)) 
                    points2.add(adsk.core.Point3D.create(xr2[i],yr2[i],0))
                if str(ch)== 'Taper Gear':
                    points1.add(adsk.core.Point3D.create(x[i]/f,y[i]/f,0)) 
                    points2.add(adsk.core.Point3D.create(x2[i]/f,y2[i]/f,0))
                if str(ch)== 'Helix Taper Gear':
                    xr.append(math.cos(tw)*x[i]+math.sin(tw)*y[i])
                    yr.append(-math.sin(tw)*x[i]+math.cos(tw)*y[i])
                    xr2.append(math.cos(tw)*x2[i]+math.sin(tw)*y2[i])
                    yr2.append(-math.sin(tw)*x2[i]+math.cos(tw)*y2[i])
                    points1.add(adsk.core.Point3D.create(xr[i]/f,yr[i]/f,0)) 
                    points2.add(adsk.core.Point3D.create(xr2[i]/f,yr2[i]/f,0))
        #------------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear' or str(ch)== 'Taper Gear':
            sk22=com.component.sketches.add(plane2)
            sk22.sketchCurves.sketchFittedSplines.add(points1)
            sk22.sketchCurves.sketchFittedSplines.add(points2)
            if str(ch)== 'Helix Taper Gear' or str(ch)== 'Taper Gear':
               sk22.sketchCurves.sketchCircles.addByCenterRadius(sk22.originPoint,da/(1.9*f))
               sk22.sketchCurves.sketchCircles.addByCenterRadius(sk22.originPoint,dd/(2*f))
            if str(ch)== 'Helix Gear':
                sk22.sketchCurves.sketchCircles.addByCenterRadius(sk22.originPoint,da/(1.9))
                sk22.sketchCurves.sketchCircles.addByCenterRadius(sk22.originPoint,dd/(2))
            sk22.sketchCurves.sketchLines.addByTwoPoints(sk22.originPoint,points1.item(0))
            sk22.sketchCurves.sketchLines.addByTwoPoints(sk22.originPoint,points2.item(0))
            sk22.arePointsShown=False
            sk22.isVisible=False
        #------------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear':
            points1=adsk.core.ObjectCollection.create()
            for i in range(0,len(xh)):
                points1.add(adsk.core.Point3D.create(xh[i],yh[i],zh[i]))
        #------------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear':
            sks=com.component.sketches.add(com.component.xYConstructionPlane)
            spl=sks.sketchCurves.sketchFittedSplines.add(points1)
            sks.arePointsShown=False
            sks.isVisible=False
        #------------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear' or str(ch)== 'Taper Gear':
            exinput=com.component.features.loftFeatures.createInput(1)
            exinput.loftSections.add(sk2.profiles.item(2))
            exinput.loftSections.add(sk22.profiles.item(2))
            if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear':
                exinput.centerLineOrRails.addRail(spl)
            loft=com.component.features.loftFeatures.add(exinput)
        if str(ch)== 'Spur Gear':
            ext=com.component.features.extrudeFeatures.addSimple(sk2.profiles.item(2),adsk.core.ValueInput.createByReal(fw),1)
        #------------------------------------------------------------------------
        finput=com.component.features.filletFeatures.createInput()
        fe=adsk.core.ObjectCollection.create()
        partedges=com.component.bRepBodies.item(0).edges
        for i in range(0,partedges.count):
            if (partedges.item(i).startVertex.geometry.z==0 and partedges.item(i).endVertex.geometry.z==fw) or (partedges.item(i).endVertex.geometry.z==0 and partedges.item(i).startVertex.geometry.z==fw):
                fe.add(partedges.item(i))
        finput.addConstantRadiusEdgeSet(fe,adsk.core.ValueInput.createByReal(fi),True)
        fillit1=com.component.features.filletFeatures.add(finput)
        #------------------------------------------------------------------------
        sk2.isVisible=False
        sk1.isVisible=False
        #------------------------------------------------------------------------
        if str(ch)== 'Helix Gear' or str(ch)== 'Helix Taper Gear' or str(ch)== 'Taper Gear':
            cobj=adsk.core.ObjectCollection.create()
            cobj.add(loft)
            cobj.add(fillit1)
            cinput=com.component.features.circularPatternFeatures.createInput(cobj,com.component.zConstructionAxis)
            cinput.quantity=adsk.core.ValueInput.createByReal(n)
            com.component.features.circularPatternFeatures.add(cinput)
        if str(ch)== 'Spur Gear':
            cobj=adsk.core.ObjectCollection.create()
            cobj.add(ext)
            cobj.add(fillit1)
            cinput=com.component.features.circularPatternFeatures.createInput(cobj,com.component.zConstructionAxis)
            cinput.quantity=adsk.core.ValueInput.createByReal(n)
            com.component.features.circularPatternFeatures.add(cinput)
        #------------------------------------------------------------------------
        skh=com.component.sketches.add(com.component.xYConstructionPlane)
        skh.sketchCurves.sketchCircles.addByCenterRadius(skh.origin,hd/2)
        points1=adsk.core.ObjectCollection.create()
        points1.add(adsk.core.Point3D.create(((hd/2)**2-(kw)**2)**0.5,kw/2,0))
        points1.add(adsk.core.Point3D.create(kh+((hd/2)**2-(kw)**2)**0.5,kw/2,0))
        points1.add(adsk.core.Point3D.create(kh+((hd/2)**2-(kw)**2)**0.5,-kw/2,0))
        points1.add(adsk.core.Point3D.create(((hd/2)**2-(kw)**2)**0.5,-kw/2,0))
        skh.sketchCurves.sketchLines.addByTwoPoints(points1.item(0),points1.item(1))
        skh.sketchCurves.sketchLines.addByTwoPoints(points1.item(1),points1.item(2))
        skh.sketchCurves.sketchLines.addByTwoPoints(points1.item(2),points1.item(3))
        skh.arePointsShown=False
        #------------------------------------------------------------------------
        hobj=adsk.core.ObjectCollection.create()
        hobj.add(skh.profiles.item(0))
        hobj.add(skh.profiles.item(1))
        com.component.features.extrudeFeatures.addSimple(hobj,adsk.core.ValueInput.createByReal(fw),1)
        skh.isVisible=False
        #------------------------------------------------------------------------
#------------------------------------------------------------------------
def CreateExternalHelixGear(x,y,x2,y2,dd,da,fw,n,tw,xh,yh,zh,fi,oh,ch):
        app = adsk.core.Application.get()
        design=adsk.fusion.Design.cast(app.activeProduct)
        root=design.rootComponent
        occ=root.occurrences
        com=occ.addNewComponent(adsk.core.Matrix3D.create())
        com.component.name='Gear'
        #-----------------------------------------------------------------------
        sk1=com.component.sketches.add(com.component.xYConstructionPlane)
        sk1.sketchCurves.sketchCircles.addByCenterRadius(sk1.origin,da/2)
        sk1.sketchCurves.sketchCircles.addByCenterRadius(sk1.origin,oh/2)
        #-----------------------------------------------------------------------
        if str(ch)== 'Internal Helix Gear':
            pls=com.component.constructionPlanes.createInput()
            pls.setByOffset(com.component.xYConstructionPlane,adsk.core.ValueInput.createByReal(fw))
            plane2=com.component.constructionPlanes.add(pls)
        #-----------------------------------------------------------------------
        com.component.features.extrudeFeatures.addSimple(sk1.profiles.item(1),adsk.core.ValueInput.createByReal(fw),3)
        #_________________________________________________________________________
        points1=adsk.core.ObjectCollection.create()
        points2=adsk.core.ObjectCollection.create()
        for i in range(0,len(x)):
            points1.add(adsk.core.Point3D.create(x[i],y[i],0))
            points2.add(adsk.core.Point3D.create(x2[i],y2[i],0))
        #------------------------------------------------------------------------
        sk2=com.component.sketches.add(com.component.xYConstructionPlane)
        sk2.sketchCurves.sketchFittedSplines.add(points1)
        sk2.sketchCurves.sketchFittedSplines.add(points2)
        sk2.sketchCurves.sketchCircles.addByCenterRadius(sk2.origin,da/1.9)
        sk2.sketchCurves.sketchCircles.addByCenterRadius(sk2.origin,dd/2)
        sk2.sketchCurves.sketchLines.addByTwoPoints(sk2.origin,points1.item(0))
        sk2.sketchCurves.sketchLines.addByTwoPoints(sk2.origin,points2.item(0))
        sk2.arePointsShown=False
        #________________________________________________________________________
        if str(ch)== 'Internal Helix Gear':
            points1=adsk.core.ObjectCollection.create()
            points2=adsk.core.ObjectCollection.create()
            xr=[]
            yr=[]
            xr2=[]
            yr2=[]
            for i in range(0,len(x)):
                xr.append(math.cos(tw)*x[i]+math.sin(tw)*y[i])
                yr.append(-math.sin(tw)*x[i]+math.cos(tw)*y[i])
                xr2.append(math.cos(tw)*x2[i]+math.sin(tw)*y2[i])
                yr2.append(-math.sin(tw)*x2[i]+math.cos(tw)*y2[i])
                points1.add(adsk.core.Point3D.create(xr[i],yr[i],0)) 
                points2.add(adsk.core.Point3D.create(xr2[i],yr2[i],0))
        #------------------------------------------------------------------------
        if str(ch)== 'Internal Helix Gear':
            sk22=com.component.sketches.add(plane2)
            sk22.sketchCurves.sketchFittedSplines.add(points1)
            sk22.sketchCurves.sketchFittedSplines.add(points2)
            sk22.sketchCurves.sketchCircles.addByCenterRadius(sk22.originPoint,da/(1.9))
            sk22.sketchCurves.sketchCircles.addByCenterRadius(sk22.originPoint,dd/(2))
            sk22.sketchCurves.sketchLines.addByTwoPoints(sk22.originPoint,points1.item(0))
            sk22.sketchCurves.sketchLines.addByTwoPoints(sk22.originPoint,points2.item(0))
            sk22.arePointsShown=False
            sk22.isVisible=False
        #------------------------------------------------------------------------
        if str(ch)== 'Internal Helix Gear':
            points1=adsk.core.ObjectCollection.create()
            for i in range(0,len(xh)):
                points1.add(adsk.core.Point3D.create(xh[i],yh[i],zh[i]))
        #------------------------------------------------------------------------
        if str(ch)== 'Internal Helix Gear':
            sks=com.component.sketches.add(com.component.xYConstructionPlane)
            spl=sks.sketchCurves.sketchFittedSplines.add(points1)
            sks.arePointsShown=False
            sks.isVisible=False
        #------------------------------------------------------------------------
        if str(ch)== 'Internal Helix Gear':
            exinput=com.component.features.loftFeatures.createInput(0)
            exinput.loftSections.add(sk2.profiles.item(2))
            exinput.loftSections.add(sk22.profiles.item(2))
            if str(ch)== 'Internal Helix Gear':
                exinput.centerLineOrRails.addRail(spl)
            loft=com.component.features.loftFeatures.add(exinput)
        if str(ch)== 'Internal Spur Gear':
            ext=com.component.features.extrudeFeatures.addSimple(sk2.profiles.item(2),adsk.core.ValueInput.createByReal(fw),0)
        #------------------------------------------------------------------------
        finput=com.component.features.filletFeatures.createInput()
        fe=adsk.core.ObjectCollection.create()
        partedges=com.component.bRepBodies.item(0).edges
        for i in range(0,partedges.count):
            if (partedges.item(i).startVertex.geometry.z==0 and partedges.item(i).endVertex.geometry.z==fw) or (partedges.item(i).endVertex.geometry.z==0 and partedges.item(i).startVertex.geometry.z==fw):
                fe.add(partedges.item(i))
        finput.addConstantRadiusEdgeSet(fe,adsk.core.ValueInput.createByReal(fi),True)
        fillit1=com.component.features.filletFeatures.add(finput)
        #------------------------------------------------------------------------
        sk2.isVisible=False
        sk1.isVisible=False
        #------------------------------------------------------------------------
        if str(ch)== 'Internal Helix Gear':
            cobj=adsk.core.ObjectCollection.create()
            cobj.add(loft)
            cobj.add(fillit1)
            cinput=com.component.features.circularPatternFeatures.createInput(cobj,com.component.zConstructionAxis)
            cinput.quantity=adsk.core.ValueInput.createByReal(n)
            com.component.features.circularPatternFeatures.add(cinput)
        if str(ch)== 'Internal Spur Gear':
            cobj=adsk.core.ObjectCollection.create()
            cobj.add(ext)
            cobj.add(fillit1)
            cinput=com.component.features.circularPatternFeatures.createInput(cobj,com.component.zConstructionAxis)
            cinput.quantity=adsk.core.ValueInput.createByReal(n)
            com.component.features.circularPatternFeatures.add(cinput)
        #------------------------------------------------------------------------

#------------------------------------------------------------------------
class MyInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = eventArgs.inputs
            list1=inputs.itemById('choice').selectedItem.name

            if str(list1) == 'Spur Gear':
               inputs.itemById('module').isVisible=True
               inputs.itemById('no').isVisible=True
               inputs.itemById('pang').isVisible=True
               inputs.itemById('nop').isVisible=True
               inputs.itemById('fwidth').isVisible=True
               inputs.itemById('fillit').isVisible=True
               inputs.itemById('holedia').isVisible=True
               inputs.itemById('keywid').isVisible=True
               inputs.itemById('keyheigh').isVisible=True
               inputs.itemById('twist').isVisible=False
               inputs.itemById('taper').isVisible=False
               inputs.itemById('outdia').isVisible=False
            elif str(list1) == 'Helix Gear':
               inputs.itemById('module').isVisible=True
               inputs.itemById('no').isVisible=True
               inputs.itemById('pang').isVisible=True
               inputs.itemById('nop').isVisible=True
               inputs.itemById('fwidth').isVisible=True
               inputs.itemById('twist').isVisible=True
               inputs.itemById('fillit').isVisible=True
               inputs.itemById('holedia').isVisible=True
               inputs.itemById('keywid').isVisible=True
               inputs.itemById('keyheigh').isVisible=True
               inputs.itemById('taper').isVisible=False
               inputs.itemById('outdia').isVisible=False
            elif  str(list1) == 'Taper Gear':
               inputs.itemById('module').isVisible=True
               inputs.itemById('no').isVisible=True
               inputs.itemById('pang').isVisible=True
               inputs.itemById('nop').isVisible=True
               inputs.itemById('fwidth').isVisible=True
               inputs.itemById('taper').isVisible=True
               inputs.itemById('fillit').isVisible=True
               inputs.itemById('holedia').isVisible=True
               inputs.itemById('keywid').isVisible=True
               inputs.itemById('keyheigh').isVisible=True
               inputs.itemById('twist').isVisible=False
               inputs.itemById('outdia').isVisible=False
            elif  str(list1) == 'Helix Taper Gear':
                inputs.itemById('module').isVisible=True
                inputs.itemById('no').isVisible=True
                inputs.itemById('pang').isVisible=True
                inputs.itemById('nop').isVisible=True
                inputs.itemById('fwidth').isVisible=True
                inputs.itemById('taper').isVisible=True
                inputs.itemById('twist').isVisible=True
                inputs.itemById('fillit').isVisible=True
                inputs.itemById('holedia').isVisible=True
                inputs.itemById('keywid').isVisible=True
                inputs.itemById('keyheigh').isVisible=True
                inputs.itemById('outdia').isVisible=False
            elif  str(list1) == 'Internal Spur Gear':
                inputs.itemById('module').isVisible=True
                inputs.itemById('no').isVisible=True
                inputs.itemById('pang').isVisible=True
                inputs.itemById('nop').isVisible=True
                inputs.itemById('fwidth').isVisible=True
                inputs.itemById('fillit').isVisible=True
                inputs.itemById('outdia').isVisible=True
                inputs.itemById('holedia').isVisible=False
                inputs.itemById('keywid').isVisible=False
                inputs.itemById('keyheigh').isVisible=False
                inputs.itemById('twist').isVisible=False
                inputs.itemById('taper').isVisible=False

            elif  str(list1) == 'Internal Helix Gear':
                inputs.itemById('module').isVisible=True
                inputs.itemById('no').isVisible=True
                inputs.itemById('pang').isVisible=True
                inputs.itemById('nop').isVisible=True
                inputs.itemById('fwidth').isVisible=True
                inputs.itemById('twist').isVisible=True
                inputs.itemById('fillit').isVisible=True
                inputs.itemById('outdia').isVisible=True
                inputs.itemById('holedia').isVisible=False
                inputs.itemById('keywid').isVisible=False
                inputs.itemById('keyheigh').isVisible=False
                inputs.itemById('taper').isVisible=False
                
              
        except:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#------------------------------------------------------------------------
class MyCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            app = adsk.core.Application.get()
            ui  = app.userInterface
            cmdf = ui.commandDefinitions.itemById('Gear_Maker')
            if cmdf:
                  cmdf.deleteMe()             
            adsk.terminate()
        except:
         if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))  
#------------------------------------------------------------------------
class MyExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
       try: 
           eventArgs = adsk.core.CommandEventArgs.cast(args)
           app = adsk.core.Application.get()
           ui  = app.userInterface
           inputs=eventArgs.command.commandInputs
           ch=inputs.itemById('choice').selectedItem.name
           m=inputs.itemById('module').value
           n=inputs.itemById('no').value
           al=inputs.itemById('pang').value
           s=inputs.itemById('nop').value
           fw=inputs.itemById('fwidth').value
           f=inputs.itemById('taper').value
           tw=inputs.itemById('twist').value
           fi=inputs.itemById('fillit').value
           hd=inputs.itemById('holedia').value
           kw=inputs.itemById('keywid').value
           kh=inputs.itemById('keyheigh').value
           oh=inputs.itemById('outdia').value
           calcualtegear(m,n,al,s,fw,f,tw,fi,hd,kw,kh,oh,ch)
           cmdf = ui.commandDefinitions.itemById('Gear_Maker')
           if cmdf:
                   cmdf.deleteMe()             
           adsk.terminate() 
       except:
         if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))  
#------------------------------------------------------------------------
class MyCommandHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            cmd=adsk.core.Command.cast(args.command)
            inputs=cmd.commandInputs
            Ddc=inputs.addDropDownCommandInput('choice','Choose Gear Type',1)
            Ddc.listItems.add('Spur Gear',True)
            Ddc.listItems.add('Helix Gear',False)
            Ddc.listItems.add('Taper Gear',False)
            Ddc.listItems.add('Helix Taper Gear',False)
            Ddc.listItems.add('Internal Spur Gear',False)
            Ddc.listItems.add('Internal Helix Gear',False)
            inputs.addValueInput('module','Enter The Module','mm',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('no','Enter The Number of teeth','',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('pang','Enter The Presssure angle','deg',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('nop','Enter No. of Points','',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('fwidth','Enter The Face width','mm',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('taper','Enter The Taber angle','deg',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('twist','Enter The Twist Angle','deg',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('fillit','Enter Fillit Radius','mm',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('holedia','Enter Hole Diameter','mm',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('keywid','Enter Key Width','mm',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('keyheigh','Enter Key Height','mm',adsk.core.ValueInput.createByReal(0))
            inputs.addValueInput('outdia','Enter Outer Diameter','mm',adsk.core.ValueInput.createByReal(0))
            inputs.itemById('module').isVisible=True
            inputs.itemById('no').isVisible=True
            inputs.itemById('pang').isVisible=True
            inputs.itemById('nop').isVisible=True
            inputs.itemById('fwidth').isVisible=True
            inputs.itemById('taper').isVisible=False
            inputs.itemById('twist').isVisible=False
            inputs.itemById('fillit').isVisible=True
            inputs.itemById('holedia').isVisible=True
            inputs.itemById('keywid').isVisible=True
            inputs.itemById('keyheigh').isVisible=True
            inputs.itemById('outdia').isVisible=False
         # when input change
            onInputChanged = MyInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)
         # Connect to the command destroyed event.
            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            handlers.append(onDestroy)
         # When OK button is pressed
            onExecute = MyExecuteHandler()
            cmd.execute.add(onExecute)
            handlers.append(onExecute)
        except:
         if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#------------------------------------------------------------------------        
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        gm=ui.commandDefinitions.itemById('Gear_Maker')
        if not gm:
                gm=ui.commandDefinitions.addButtonDefinition('Gear_Maker','Any_gear_maker','press Me')
        onCmd = MyCommandHandler()
        gm.commandCreated.add(onCmd)
        handlers.append(onCmd)
        gm.execute()
        adsk.autoTerminate(False)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#------------------------------------------------------------------------
def stop(context):
    try:    
        
        # Delete the command definition.
        cmdf = ui.commandDefinitions.itemById('Spline_Derived_Euations')
        if cmdf:
            cmdf.deleteMe()
            
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
    
