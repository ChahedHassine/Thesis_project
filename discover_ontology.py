from rdflib import Graph
from rdflib.namespace import CSVW, DC, DCAT, DCTERMS, DOAP, FOAF, ODRL2, ORG, OWL, PROF, PROV, RDF, RDFS, SDO, SH, SKOS, SOSA, SSN, TIME, VOID, XMLNS, XSD

from rdflib import URIRef

g = Graph()

g.parse("robot.ntriples")

print("--- printing mboxes ---")
g = Graph()
g.parse("robot.ntriples")
camera=URIRef("http://example.org/camera")
for c in g.subjects(RDF.type, camera):
        print(c.split('://')[1])
       