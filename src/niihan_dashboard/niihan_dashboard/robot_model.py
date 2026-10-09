"""Transmit the actual URDF visual hierarchy without a second hand-built rover."""
import xml.etree.ElementTree as ET

def vector(value, default):
    return [float(v) for v in value.split()] if value else default

def origin(element):
    node=element.find('origin')
    return {'xyz': vector(node.get('xyz'), [0.,0.,0.]),
            'rpy': vector(node.get('rpy'), [0.,0.,0.])} if node is not None else {'xyz':[0.,0.,0.], 'rpy':[0.,0.,0.]}

def visual_model(xml):
    root=ET.fromstring(xml)
    materials={m.get('name'):vector(m.find('color').get('rgba'), [0.5,0.5,0.5,1.])
               for m in root.findall('material') if m.find('color') is not None}
    links=[]
    for link in root.findall('link'):
        visuals=[]
        for visual in link.findall('visual'):
            geometry=visual.find('geometry')
            shape=next(iter(geometry), None) if geometry is not None else None
            if shape is None:continue
            if shape.tag=='box':spec={'kind':'box','size':vector(shape.get('size'),[1.,1.,1.])}
            elif shape.tag=='cylinder':spec={'kind':'cylinder','radius':float(shape.get('radius')),'length':float(shape.get('length'))}
            elif shape.tag=='sphere':spec={'kind':'sphere','radius':float(shape.get('radius'))}
            else:raise ValueError('Unsupported URDF visual geometry: '+shape.tag)
            material=visual.find('material');rgba=[0.5,0.5,0.5,1.]
            if material is not None:
                color=material.find('color')
                rgba=vector(color.get('rgba'),rgba) if color is not None else materials.get(material.get('name'),rgba)
            visuals.append({**origin(visual),'geometry':spec,'rgba':rgba})
        links.append({'name':link.get('name'),'visuals':visuals})
    joints=[{'name':j.get('name'),'parent':j.find('parent').get('link'),
             'child':j.find('child').get('link'),'kind':j.get('type'),
             'axis':vector(j.find('axis').get('xyz'),[1.,0.,0.]) if j.find('axis') is not None else [1.,0.,0.],**origin(j)} for j in root.findall('joint')]
    children={j['child'] for j in joints};roots=[l['name'] for l in links if l['name'] not in children]
    if len(roots)!=1:raise ValueError('URDF must have one root link')
    return {'type':'robot_model','name':root.get('name'),'root':roots[0],'links':links,'joints':joints}
