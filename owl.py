from email.errors import HeaderMissingRequiredValue

from rdflib import Graph, Literal, RDF, URIRef

# rdflib knows about quite a few popular namespaces, like W3C ontologies, schema.org etc.

from rdflib.namespace import CSVW, DC, DCAT, DCTERMS, DOAP, FOAF, ODRL2, ORG, OWL, \
                           PROF, PROV, RDF, RDFS, SDO, SH, SKOS, SOSA, SSN, TIME, \
                           VOID, XMLNS, XSD

# Create a Graph of pieces and cameras

g = Graph()
group1 = URIRef("http://example.org/group1")
# Add triples using store's add() method.
g.add((group1, RDF.type, OWL.Class))
group2 = URIRef("http://example.org/group2")
# Add triples using store's add() method.
g.add((group2, RDF.type, OWL.Class))
# Create an RDF URI node to use as the subject for multiple triples
piece1 = URIRef("http://example.org/piece0")
# Add triples using store's add() method.
g.add((piece1, RDF.type, group1))
# Create an RDF URI node to use as the subject for multiple triples
piece2 = URIRef("http://example.org/piece5")
# Add triples using store's add() method.
g.add((piece2, RDF.type, group1))
# Create an RDF URI node to use as the subject for multiple triples
piece4 = URIRef("http://example.org/piece1")
# Add triples using store's add() method.
g.add((piece4, RDF.type, group1))
# Create an RDF URI node to use as the subject for multiple triples
piece5 = URIRef("http://example.org/piece3")
# Add triples using store's add() method.
g.add((piece5, RDF.type, group1))

camera = URIRef("http://example.org/camera")
# Add triples using store's add() method.
g.add((camera, RDF.type, OWL.Class))

# Create an RDF URI node to use as the subject for multiple triples
camera_1 = URIRef("http://move_artifact@comnets-pc08")
# Add triples using store's add() method.
g.add((camera_1, RDF.type, camera))
# Create an RDF URI node to use as the subject for multiple triples
camera_2 = URIRef("http://move_artifact2@comnets-pc08")
# Add triples using store's add() method.
g.add((camera_2, RDF.type, camera))


# print all the data in the Notation3 format
print("--- printing ontology ---")
# Print out the entire Graph in the RDF Turtle format
print(g.serialize(format="turtle"))
g.serialize(destination="robot.ntriples")